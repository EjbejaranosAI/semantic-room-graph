"""
Goal Resolver Node
Convert the selected region from query candidates into a final navigation
goal pose. Publishes a PoseStamped suitable for Nav2.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from semantic_map_msgs.msg import QueryResult, SemanticGraph


class GoalResolverNode(Node):
    def __init__(self):
        super().__init__('goal_resolver_node')

        self.graph = None

        self.sub_graph = self.create_subscription(
            SemanticGraph, '/semantic_map/semantic_graph', self._on_graph, 10
        )
        self.sub_candidates = self.create_subscription(
            QueryResult, '/semantic_map/query_candidates', self._on_candidates, 10
        )

        self.pub_goal = self.create_publisher(
            PoseStamped, '/semantic_map/final_goal', 10
        )

        self.get_logger().info('GoalResolverNode ready.')

    def _on_graph(self, msg: SemanticGraph):
        self.graph = msg

    def _on_candidates(self, msg: QueryResult):
        if not msg.success:
            self.get_logger().warn(
                f'Query failed: {msg.message}'
            )
            return

        region_id = msg.selected_region_id
        goal_pose = msg.selected_goal_pose

        if self.graph is not None:
            for r in self.graph.regions:
                if r.region_id == region_id:
                    goal_pose = r.entry_pose
                    break

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose = goal_pose

        self.pub_goal.publish(goal)
        self.get_logger().info(
            f'Goal resolved: region {region_id} -> '
            f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalResolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
