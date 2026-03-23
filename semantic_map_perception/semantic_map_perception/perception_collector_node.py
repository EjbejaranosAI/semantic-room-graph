"""
Perception Collector Node
Capture RGB observations when the robot reaches a region pose.
Listens for region_visit_event, grabs the latest camera image, and
publishes a RegionObservation message.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from semantic_map_msgs.msg import RegionObservation


class PerceptionCollectorNode(Node):
    def __init__(self):
        super().__init__('perception_collector_node')

        self.declare_parameter('camera_topic', '/camera/rgb/image_raw')
        self.declare_parameter('capture_delay_sec', 1.0)

        camera_topic = self.get_parameter('camera_topic').value
        self.capture_delay = self.get_parameter('capture_delay_sec').value
        self.camera_topic = camera_topic

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_pose = None

        self.sub_visit = self.create_subscription(
            Int32, '/semantic_map/region_visit_event', self._on_visit, 10
        )
        self.sub_image = self.create_subscription(
            Image, camera_topic, self._on_image, 10
        )
        self.sub_pose = self.create_subscription(
            PoseStamped, '/robot_pose', self._on_pose, 10
        )

        self.pub_observation = self.create_publisher(
            RegionObservation, '/semantic_map/region_observation', 10
        )

        self.get_logger().info('PerceptionCollectorNode ready.')

    def _on_image(self, msg: Image):
        self.latest_image = msg

    def _on_pose(self, msg: PoseStamped):
        self.latest_pose = msg

    def _on_visit(self, msg: Int32):
        region_id = msg.data

        timer = self.create_timer(
            self.capture_delay,
            lambda: self._capture_once(region_id, timer),
        )

    def _capture_once(self, region_id: int, timer):
        timer.cancel()
        self.destroy_timer(timer)

        if self.latest_image is None:
            self.get_logger().warn(
                f'No image available for region {region_id} on {self.camera_topic}.'
            )
            return

        obs = RegionObservation()
        obs.header.stamp = self.get_clock().now().to_msg()
        obs.header.frame_id = 'map'
        obs.region_id = region_id
        obs.rgb_image = self.latest_image

        if self.latest_pose is not None:
            obs.robot_pose = self.latest_pose.pose
        obs.frame_id = 'map'

        self.pub_observation.publish(obs)
        self.get_logger().info(f'Published observation for region {region_id}')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionCollectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
