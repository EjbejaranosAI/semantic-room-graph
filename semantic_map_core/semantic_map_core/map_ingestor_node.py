"""
Map Ingestor Node
Subscribe to /map (OccupancyGrid) and convert to a binary free-space image
plus map metadata for downstream segmentation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Float32MultiArray
from cv_bridge import CvBridge


class MapIngestorNode(Node):
    def __init__(self):
        super().__init__('map_ingestor_node')

        self.declare_parameter('free_threshold', 50)
        self.declare_parameter('map_topic', '/map')

        self.free_thresh = self.get_parameter('free_threshold').value
        map_topic = self.get_parameter('map_topic').value

        self.bridge = CvBridge()
        self.map_received = False

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.sub_map = self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, map_qos
        )

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.pub_grid_image = self.create_publisher(
            Image, '/semantic_map/grid_image', latched_qos
        )
        self.pub_map_metadata = self.create_publisher(
            Float32MultiArray, '/semantic_map/map_metadata', latched_qos
        )

        self.get_logger().info('MapIngestorNode ready, waiting for /map ...')

    def _on_map(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))

        binary = np.zeros((h, w), dtype=np.uint8)
        binary[(grid >= 0) & (grid < self.free_thresh)] = 255

        img_msg = self.bridge.cv2_to_imgmsg(binary, encoding='mono8')
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = msg.header.frame_id
        self.pub_grid_image.publish(img_msg)

        meta = Float32MultiArray()
        meta.data = [
            float(w),
            float(h),
            float(resolution),
            float(origin.position.x),
            float(origin.position.y),
            float(origin.orientation.z),
            float(origin.orientation.w),
        ]
        self.pub_map_metadata.publish(meta)

        if not self.map_received:
            self.get_logger().info(
                f'Map received: {w}x{h}, resolution={resolution:.3f} m/px'
            )
            self.map_received = True


def main(args=None):
    rclpy.init(args=args)
    node = MapIngestorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
