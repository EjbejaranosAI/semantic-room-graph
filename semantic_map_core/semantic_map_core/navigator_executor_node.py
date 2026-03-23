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

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Int32MultiArray
from nav2_msgs.action import NavigateToPose

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
        self.goal_timeout = self.get_parameter('goal_timeout_sec').value
        self.use_inspection_fallback = self.get_parameter('use_inspection_fallback').value

        self.cb_group = ReentrantCallbackGroup()
        self.graph = None
        self.exploration_plan = []
        self.current_goal_idx = 0
        self.navigating = False
        self.server_available = False
        self.inspection_attempted = set()
        self.active_goal_uses_inspection = False

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

    def _on_plan(self, msg: Int32MultiArray):
        self.exploration_plan = list(msg.data)
        self.current_goal_idx = 0
        self.inspection_attempted.clear()
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

        use_inspection = (
            self.use_inspection_fallback and region_id in self.inspection_attempted
        )
        goal_pose = region.inspection_pose if use_inspection else region.entry_pose

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = goal_pose
        self.active_goal_uses_inspection = use_inspection

        self.get_logger().info(
            f'Navigating to region {region_id} '
            f'({"inspection" if use_inspection else "entry"} pose: '
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
            if self.use_inspection_fallback and not self.active_goal_uses_inspection:
                self.get_logger().warn(
                    f'Retrying region {region_id} with inspection pose.'
                )
                self.inspection_attempted.add(region_id)
                self.navigating = False
                self._send_next_goal()
                return
            self.navigating = False
            self.current_goal_idx += 1
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
        else:
            self.get_logger().warn(
                f'Navigation to region {region_id} failed (status={result.status})'
            )
            if self.use_inspection_fallback and not self.active_goal_uses_inspection:
                self.get_logger().warn(
                    f'Retrying region {region_id} with inspection pose after failure.'
                )
                self.inspection_attempted.add(region_id)
                self.navigating = False
                self._send_next_goal()
                return

        self.navigating = False
        self.current_goal_idx += 1
        self._send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
