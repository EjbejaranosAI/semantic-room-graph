"""
Exploration Planner Node
Compute a visit order over unexplored regions using a nearest-neighbor
heuristic from the robot's current position. Publishes the ordered plan
for the navigator executor.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray

from semantic_map_msgs.msg import SemanticGraph

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class ExplorationPlannerNode(Node):
    def __init__(self):
        super().__init__('exploration_planner_node')

        self.declare_parameter('replan_on_update', True)
        self.declare_parameter('visit_corridors', False)

        self.replan_on_update = self.get_parameter('replan_on_update').value
        self.visit_corridors = self.get_parameter('visit_corridors').value
        self.graph = None
        self.robot_pose = None

        self.sub_graph = self.create_subscription(
            SemanticGraph, '/semantic_map/graph_with_poses', self._on_graph, _LATCHED_QOS
        )
        self.sub_pose = self.create_subscription(
            PoseStamped, '/robot_pose', self._on_pose, 10
        )

        self.pub_plan = self.create_publisher(
            Int32MultiArray, '/semantic_map/exploration_plan', _LATCHED_QOS
        )

        self.get_logger().info('ExplorationPlannerNode ready.')

    def _on_pose(self, msg: PoseStamped):
        self.robot_pose = msg

    def _on_graph(self, msg: SemanticGraph):
        self.graph = msg
        if self.replan_on_update:
            self._plan()

    def _plan(self):
        if self.graph is None:
            return

        unexplored = [
            r for r in self.graph.regions if r.status == 'unexplored'
        ]
        if not self.visit_corridors:
            room_candidates = [
                r for r in unexplored if r.region_type_geom.lower() == 'room'
            ]
            if room_candidates:
                unexplored = room_candidates
            else:
                self.get_logger().warn(
                    'No room-like regions found; falling back to all unexplored regions.'
                )

        if not unexplored:
            self.get_logger().info('All regions explored. No plan to generate.')
            plan = Int32MultiArray()
            plan.data = []
            self.pub_plan.publish(plan)
            return

        if self.robot_pose is not None:
            start_x = self.robot_pose.pose.position.x
            start_y = self.robot_pose.pose.position.y
        else:
            start_x = unexplored[0].entry_pose.position.x
            start_y = unexplored[0].entry_pose.position.y

        order = []
        remaining = list(unexplored)
        cx, cy = start_x, start_y

        while remaining:
            best_idx = 0
            best_dist = float('inf')
            for i, r in enumerate(remaining):
                dx = r.entry_pose.position.x - cx
                dy = r.entry_pose.position.y - cy
                d = math.hypot(dx, dy)
                if d < best_dist:
                    best_dist = d
                    best_idx = i

            chosen = remaining.pop(best_idx)
            order.append(chosen.region_id)
            cx = chosen.entry_pose.position.x
            cy = chosen.entry_pose.position.y

        plan = Int32MultiArray()
        plan.data = order
        self.pub_plan.publish(plan)
        self.get_logger().info(
            f'Exploration plan: {len(order)} regions to visit: {order}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
