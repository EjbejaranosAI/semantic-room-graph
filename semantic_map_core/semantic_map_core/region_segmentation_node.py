"""
Region Segmentation Node
Segment free space from the binary grid image into topological regions using
morphological operations and watershed-based partitioning. Publishes the raw
segmented regions as a labeled image + region metadata.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from semantic_map_msgs.msg import RegionNode, SemanticGraph


class RegionSegmentationNode(Node):
    def __init__(self):
        super().__init__('region_segmentation_node')

        self.declare_parameter('morph_kernel_size', 5)
        self.declare_parameter('min_region_area', 500)
        self.declare_parameter('distance_threshold_ratio', 0.3)

        self.morph_k = self.get_parameter('morph_kernel_size').value
        self.min_area = self.get_parameter('min_region_area').value
        self.dist_ratio = self.get_parameter('distance_threshold_ratio').value

        self.bridge = CvBridge()
        self.grid_image = None
        self.map_meta = None

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.sub_grid = self.create_subscription(
            Image, '/semantic_map/grid_image', self._on_grid, latched_qos
        )
        self.sub_meta = self.create_subscription(
            Float32MultiArray, '/semantic_map/map_metadata', self._on_meta, latched_qos
        )

        self.pub_regions = self.create_publisher(
            SemanticGraph, '/semantic_map/regions_raw', 10
        )
        self.pub_label_image = self.create_publisher(
            Image, '/semantic_map/label_image', 10
        )

        self.get_logger().info('RegionSegmentationNode ready.')

    def _on_meta(self, msg: Float32MultiArray):
        self.map_meta = msg.data
        self._try_segment()

    def _on_grid(self, msg: Image):
        self.grid_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self._try_segment()

    def _try_segment(self):
        if self.grid_image is not None and self.map_meta is not None:
            self._segment()

    def _segment(self):
        binary = self.grid_image.copy()
        kernel = np.ones((self.morph_k, self.morph_k), np.uint8)

        opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=2)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel, iterations=2)

        dist_transform = cv2.distanceTransform(closed, cv2.DIST_L2, 5)
        _, sure_fg = cv2.threshold(
            dist_transform,
            self.dist_ratio * dist_transform.max(),
            255,
            cv2.THRESH_BINARY,
        )
        sure_fg = np.uint8(sure_fg)

        sure_bg = cv2.dilate(closed, kernel, iterations=3)
        unknown = cv2.subtract(sure_bg, sure_fg)

        num_labels, markers = cv2.connectedComponents(sure_fg)
        markers = markers + 1
        markers[unknown == 255] = 0

        color_img = cv2.cvtColor(closed, cv2.COLOR_GRAY2BGR)
        markers = cv2.watershed(color_img, markers)

        meta = self.map_meta
        resolution = meta[2]
        origin_x = meta[3]
        origin_y = meta[4]

        region_nodes = []
        valid_id = 0

        for label_val in range(2, num_labels + 1):
            mask = (markers == label_val).astype(np.uint8)
            area = int(np.sum(mask))

            if area < self.min_area:
                continue

            ys, xs = np.where(mask > 0)
            cx = float(np.mean(xs))
            cy = float(np.mean(ys))

            wx = origin_x + cx * resolution
            wy = origin_y + (mask.shape[0] - cy) * resolution

            node = RegionNode()
            node.region_id = valid_id
            node.region_type_geom = self._classify_shape(mask)
            node.status = 'unexplored'
            node.semantic_label = ''
            node.label_confidence = 0.0
            node.entry_pose.position.x = wx
            node.entry_pose.position.y = wy
            node.entry_pose.position.z = 0.0
            node.entry_pose.orientation.w = 1.0
            node.ocr_text = []
            node.neighbors = []

            region_nodes.append(node)
            valid_id += 1

        graph = SemanticGraph()
        graph.header.stamp = self.get_clock().now().to_msg()
        graph.header.frame_id = 'map'
        graph.regions = region_nodes
        graph.edges = []
        graph.map_frame = 'map'
        graph.map_resolution = resolution

        self.pub_regions.publish(graph)

        label_vis = np.zeros_like(binary)
        for label_val in range(2, num_labels + 1):
            label_vis[markers == label_val] = min(
                255, int((label_val - 1) * (255 // max(num_labels - 1, 1)))
            )
        label_msg = self.bridge.cv2_to_imgmsg(label_vis, encoding='mono8')
        label_msg.header.stamp = self.get_clock().now().to_msg()
        label_msg.header.frame_id = 'map'
        self.pub_label_image.publish(label_msg)

        self.get_logger().info(
            f'Segmentation complete: {valid_id} regions from {num_labels - 1} raw components'
        )

    def _classify_shape(self, mask: np.ndarray) -> str:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 'unknown'
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)
        if perimeter < 1e-3:
            return 'unknown'

        circularity = 4.0 * np.pi * area / (perimeter * perimeter)
        rect = cv2.minAreaRect(cnt)
        w, h = rect[1]
        if min(w, h) < 1e-3:
            return 'unknown'
        aspect = max(w, h) / min(w, h)

        if aspect > 3.0:
            return 'corridor'
        elif circularity > 0.7:
            return 'room'
        else:
            return 'room'


def main(args=None):
    rclpy.init(args=args)
    node = RegionSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
