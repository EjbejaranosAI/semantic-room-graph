"""
Task Navigator Node
Execute the final user-requested navigation goal through Nav2
NavigateToPose action. Reports success/failure on task_result.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


class TaskNavigatorNode(Node):
    def __init__(self):
        super().__init__('task_navigator_node')

        self.declare_parameter('goal_timeout_sec', 180.0)
        self.goal_timeout = self.get_parameter('goal_timeout_sec').value

        self.cb_group = ReentrantCallbackGroup()
        self.navigating = False

        self.sub_goal = self.create_subscription(
            PoseStamped, '/semantic_map/final_goal', self._on_goal, 10
        )

        self.pub_result = self.create_publisher(
            String, '/semantic_map/task_result', 10
        )

        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group
        )

        self.get_logger().info('TaskNavigatorNode ready.')

    def _on_goal(self, msg: PoseStamped):
        if self.navigating:
            self.get_logger().warn('Already navigating, ignoring new goal.')
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg

        self.get_logger().info(
            f'Navigating to goal: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})'
        )

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            self._publish_result(False, 'Nav2 not available')
            return

        self.navigating = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self.navigating = False
            self._publish_result(False, 'Goal rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result()
        self.navigating = False

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded!')
            self._publish_result(True, 'Arrived at destination')
        else:
            self.get_logger().warn(
                f'Navigation failed with status {result.status}'
            )
            self._publish_result(False, f'Navigation failed (status={result.status})')

    def _publish_result(self, success: bool, message: str):
        msg = String()
        msg.data = f'{{"success": {str(success).lower()}, "message": "{message}"}}'
        self.pub_result.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskNavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
