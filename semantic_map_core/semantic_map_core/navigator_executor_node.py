"""
Navigator Executor Node
Execute exploration goals through Nav2 NavigateToPose action.
Iterates through the exploration plan, sends each region's entry_pose
as a goal, and publishes region_visit_event on arrival.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import math
import numpy as np

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Int32MultiArray, Float32MultiArray
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from semantic_map_msgs.msg import SemanticGraph

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class NavigatorExecutorNode(Node):
    def __init__(self):
        super().__init__('navigator_executor_node')

        self.declare_parameter('goal_timeout_sec', 120.0)
        self.declare_parameter('use_inspection_fallback', True)
        self.declare_parameter('candidate_radius_m', 0.7)
        self.declare_parameter('max_goal_attempts_per_region', 8)
        self.declare_parameter('goal_clearance_px', 2)
        self.goal_timeout = self.get_parameter('goal_timeout_sec').value
        self.use_inspection_fallback = self.get_parameter('use_inspection_fallback').value
        self.candidate_radius_m = self.get_parameter('candidate_radius_m').value
        self.max_goal_attempts = self.get_parameter('max_goal_attempts_per_region').value
        self.goal_clearance_px = self.get_parameter('goal_clearance_px').value

        self.cb_group = ReentrantCallbackGroup()
        self.graph = None
        self.grid_image = None
        self.map_meta = None
        self.bridge = CvBridge()
        self.exploration_plan = []
        self.current_goal_idx = 0
        self.navigating = False
        self.server_available = False
        self.goal_candidates = {}
        self.goal_attempt_idx = {}
        self.failed_regions = set()
        self.active_goal_candidate_idx = 0

        self.sub_graph = self.create_subscription(
            SemanticGraph, '/semantic_map/graph_with_poses',
            self._on_graph, _LATCHED_QOS,
            callback_group=self.cb_group,
        )
        self.sub_plan = self.create_subscription(
            Int32MultiArray, '/semantic_map/exploration_plan',
            self._on_plan, _LATCHED_QOS,
            callback_group=self.cb_group,
        )
        self.sub_grid = self.create_subscription(
            Image, '/semantic_map/grid_image', self._on_grid, _LATCHED_QOS
        )
        self.sub_meta = self.create_subscription(
            Float32MultiArray, '/semantic_map/map_metadata', self._on_meta, _LATCHED_QOS
        )

        self.pub_visit_event = self.create_publisher(
            Int32, '/semantic_map/region_visit_event', 10
        )

        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group,
        )

        self.get_logger().info('NavigatorExecutorNode ready.')

    def _on_graph(self, msg: SemanticGraph):
        self.graph = msg
        self.get_logger().info(
            f'Graph received: {len(msg.regions)} regions'
        )
        self._try_navigate()

    def _on_grid(self, msg: Image):
        self.grid_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

    def _on_meta(self, msg: Float32MultiArray):
        self.map_meta = msg.data

    def _on_plan(self, msg: Int32MultiArray):
        self.exploration_plan = list(msg.data)
        self.current_goal_idx = 0
        self.goal_candidates.clear()
        self.goal_attempt_idx.clear()
        self.failed_regions.clear()
        self.get_logger().info(
            f'Received exploration plan: {self.exploration_plan}'
        )
        self._try_navigate()

    def _try_navigate(self):
        if self.graph is None:
            self.get_logger().info('Waiting for graph before navigating...')
            return
        if not self.exploration_plan:
            return
        if not self.navigating:
            self._send_next_goal()

    def _find_region(self, region_id: int):
        if self.graph is None:
            return None
        for r in self.graph.regions:
            if r.region_id == region_id:
                return r
        return None

    def _send_next_goal(self):
        if self.current_goal_idx >= len(self.exploration_plan):
            if self.failed_regions:
                self.get_logger().warn(
                    f'Exploration finished with failed regions: '
                    f'{sorted(self.failed_regions)}'
                )
            else:
                self.get_logger().info('Exploration complete: all regions visited.')
            self.navigating = False
            return

        if self.graph is None:
            self.get_logger().warn('Graph not available yet, deferring...')
            return

        region_id = self.exploration_plan[self.current_goal_idx]
        region = self._find_region(region_id)

        if region is None:
            self.get_logger().warn(f'Region {region_id} not found, skipping.')
            self.current_goal_idx += 1
            self._send_next_goal()
            return

        if region_id not in self.goal_candidates:
            self.goal_candidates[region_id] = self._build_goal_candidates(region)
            self.goal_attempt_idx[region_id] = 0

        candidates = self.goal_candidates.get(region_id, [])
        attempt_idx = self.goal_attempt_idx.get(region_id, 0)
        if attempt_idx >= len(candidates) or attempt_idx >= self.max_goal_attempts:
            self.get_logger().warn(
                f'No valid goal candidates left for region {region_id}; skipping.'
            )
            self.failed_regions.add(region_id)
            self.current_goal_idx += 1
            self._send_next_goal()
            return
        goal_pose = candidates[attempt_idx]
        self.active_goal_candidate_idx = attempt_idx

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = goal_pose

        self.get_logger().info(
            f'Navigating to region {region_id} '
            f'(candidate {attempt_idx + 1}/{len(candidates)}: '
            f'{goal_pose.position.x:.2f}, {goal_pose.position.y:.2f})'
        )

        if not self.server_available:
            self.get_logger().info('Waiting for Nav2 action server...')
            if not self.nav_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error(
                    'Nav2 action server not available after 10s! '
                    'Retrying in 5s...'
                )
                self.create_timer(5.0, self._retry_navigate_once)
                return
            self.server_available = True

        self.navigating = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _retry_navigate_once(self):
        """One-shot retry triggered by timer."""
        self._send_next_goal()

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        region_id = self.exploration_plan[self.current_goal_idx]
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal rejected by Nav2 for region {region_id}.')
            self.goal_attempt_idx[region_id] = self.active_goal_candidate_idx + 1
            self.navigating = False
            self._send_next_goal()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result()
        region_id = self.exploration_plan[self.current_goal_idx]

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f'Reached region {region_id}')
            visit_msg = Int32()
            visit_msg.data = region_id
            self.pub_visit_event.publish(visit_msg)
            self.goal_candidates.pop(region_id, None)
            self.goal_attempt_idx.pop(region_id, None)
        else:
            self.get_logger().warn(
                f'Navigation to region {region_id} failed (status={result.status})'
            )
            self.goal_attempt_idx[region_id] = self.active_goal_candidate_idx + 1
            self.navigating = False
            self._send_next_goal()
            return

        self.navigating = False
        self.current_goal_idx += 1
        self._send_next_goal()

    def _build_goal_candidates(self, region):
        seeds = [region.entry_pose]
        if self.use_inspection_fallback:
            seeds.append(region.inspection_pose)

        candidates = []
        radial = [0.0, self.candidate_radius_m, -self.candidate_radius_m]
        lateral = [0.0, self.candidate_radius_m * 0.6, -self.candidate_radius_m * 0.6]
        for seed in seeds:
            for dx in radial:
                for dy in lateral:
                    p = PoseStamped().pose
                    p.position.x = seed.position.x + dx
                    p.position.y = seed.position.y + dy
                    p.position.z = 0.0
                    p.orientation = seed.orientation
                    if self._is_world_pose_free(p.position.x, p.position.y):
                        if not self._is_duplicate_pose(candidates, p):
                            candidates.append(p)

        if not candidates:
            if self._is_world_pose_free(region.entry_pose.position.x, region.entry_pose.position.y):
                candidates.append(region.entry_pose)
            elif self._is_world_pose_free(
                region.inspection_pose.position.x, region.inspection_pose.position.y
            ):
                candidates.append(region.inspection_pose)
        return candidates

    def _is_duplicate_pose(self, poses, pose, min_dist=0.25):
        for p in poses:
            if math.hypot(p.position.x - pose.position.x, p.position.y - pose.position.y) < min_dist:
                return True
        return False

    def _is_world_pose_free(self, wx, wy):
        if self.grid_image is None or self.map_meta is None or len(self.map_meta) < 5:
            return True
        h, w = self.grid_image.shape
        resolution = float(self.map_meta[2])
        if resolution <= 0:
            return True
        origin_x = float(self.map_meta[3])
        origin_y = float(self.map_meta[4])
        px = int((wx - origin_x) / resolution)
        py = h - 1 - int((wy - origin_y) / resolution)
        if px < 0 or px >= w or py < 0 or py >= h:
            return False
        r = int(max(0, self.goal_clearance_px))
        x0 = max(0, px - r)
        x1 = min(w, px + r + 1)
        y0 = max(0, py - r)
        y1 = min(h, py + r + 1)
        patch = self.grid_image[y0:y1, x0:x1]
        return bool(np.all(patch > 0))


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
