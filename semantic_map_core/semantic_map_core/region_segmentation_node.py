"""
Region Segmentation Node
Segment free space from the binary grid image into topological regions.

Strategy:
  1. Clean the binary free-space image with morphological opening.
  2. Try increasing erosion radii until the free space splits into >=2
     connected components (rooms separated at doorways).
  3. Use those components as seeds for OpenCV watershed to grow regions
     back into the full free space.
  4. Fallback: if no erosion separates the space, subdivide into a regular
     grid of navigable cells so the exploration planner still has goals.
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
        self.declare_parameter('erosion_min_m', 0.3)
        self.declare_parameter('erosion_max_m', 2.0)
        self.declare_parameter('erosion_step_m', 0.15)
        self.declare_parameter('grid_cell_size_m', 3.0)
        self.declare_parameter('exclude_border_regions', True)
        self.declare_parameter('border_margin_px', 2)

        self.morph_k = self.get_parameter('morph_kernel_size').value
        self.min_area = self.get_parameter('min_region_area').value
        self.erosion_min_m = self.get_parameter('erosion_min_m').value
        self.erosion_max_m = self.get_parameter('erosion_max_m').value
        self.erosion_step_m = self.get_parameter('erosion_step_m').value
        self.grid_cell_m = self.get_parameter('grid_cell_size_m').value
        self.exclude_border_regions = self.get_parameter('exclude_border_regions').value
        self.border_margin_px = self.get_parameter('border_margin_px').value

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
            SemanticGraph, '/semantic_map/regions_raw', latched_qos
        )
        self.pub_label_image = self.create_publisher(
            Image, '/semantic_map/label_image', latched_qos
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

    # ------------------------------------------------------------------
    def _segment(self):
        binary = self.grid_image.copy()
        kernel = np.ones((self.morph_k, self.morph_k), np.uint8)
        clean = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=2)

        meta = self.map_meta
        resolution = float(meta[2])
        origin_x = float(meta[3])
        origin_y = float(meta[4])
        h = clean.shape[0]

        cc_labels, valid_labels, info = self._find_room_seeds(clean, resolution)

        self.get_logger().info(f'Seed search: {info}')

        if not valid_labels:
            self.get_logger().warn('No valid regions found — skipping.')
            return

        if len(valid_labels) == 1:
            markers = cc_labels.copy().astype(np.int32)
            ws_label_offset = 0
        else:
            markers = np.zeros(clean.shape, dtype=np.int32)
            markers[clean == 0] = 1
            for new_id, old_lbl in enumerate(valid_labels, start=2):
                markers[cc_labels == old_lbl] = new_id
            color = cv2.cvtColor(clean, cv2.COLOR_GRAY2BGR)
            markers = cv2.watershed(color, markers)
            ws_label_offset = 2

        region_nodes = []
        for idx, _ in enumerate(valid_labels):
            ws_label = idx + ws_label_offset if ws_label_offset else valid_labels[idx]
            mask = (markers == ws_label).astype(np.uint8)
            area = int(np.sum(mask))
            if area < self.min_area:
                continue
            if self.exclude_border_regions and self._touches_border(mask):
                self.get_logger().debug(
                    f'Discarding border region (label={ws_label}, area={area})'
                )
                continue

            ys, xs = np.where(mask > 0)
            cx, cy = float(np.mean(xs)), float(np.mean(ys))
            wx = origin_x + cx * resolution
            wy = origin_y + (h - cy) * resolution

            node = RegionNode()
            node.region_id = len(region_nodes)
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

        graph = SemanticGraph()
        graph.header.stamp = self.get_clock().now().to_msg()
        graph.header.frame_id = 'map'
        graph.regions = region_nodes
        graph.edges = []
        graph.map_frame = 'map'
        graph.map_resolution = resolution
        self.pub_regions.publish(graph)

        n_regions = len(region_nodes)
        label_vis = np.zeros_like(binary)
        for idx in range(n_regions):
            ws_label = idx + ws_label_offset if ws_label_offset else valid_labels[idx]
            intensity = int((idx + 1) * (255 // max(n_regions, 1)))
            label_vis[markers == ws_label] = min(255, max(1, intensity))
        label_msg = self.bridge.cv2_to_imgmsg(label_vis, encoding='mono8')
        label_msg.header.stamp = self.get_clock().now().to_msg()
        label_msg.header.frame_id = 'map'
        self.pub_label_image.publish(label_msg)

        self.get_logger().info(f'Segmentation complete: {n_regions} regions')

    # ------------------------------------------------------------------
    def _find_room_seeds(self, clean, resolution):
        """Try increasing erosion to disconnect rooms at doorways."""
        min_e = max(3, int(self.erosion_min_m / resolution))
        max_e = max(min_e + 2, int(self.erosion_max_m / resolution))
        step = max(1, int(self.erosion_step_m / resolution))
        best = None

        for e_px in range(min_e, max_e + 1, step):
            k = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * e_px + 1, 2 * e_px + 1)
            )
            eroded = cv2.erode(clean, k, iterations=1)
            n_cc, cc_labels = cv2.connectedComponents(eroded)

            valid = [
                lbl for lbl in range(1, n_cc)
                if np.sum(cc_labels == lbl) >= self.min_area
            ]

            if len(valid) >= 2:
                candidate = (
                    cc_labels,
                    valid,
                    e_px,
                    f'{len(valid)} seeds via erosion '
                    f'{e_px}px ({e_px * resolution:.2f}m)'
                )
                if best is None or len(valid) > len(best[1]):
                    best = candidate

        if best is not None:
            return best[0], best[1], best[3]
        return self._grid_fallback(clean, resolution)

    def _grid_fallback(self, clean, resolution):
        """Subdivide free space into navigable grid cells."""
        h, w = clean.shape
        cell_px = max(10, int(self.grid_cell_m / resolution))

        grid_labels = np.zeros((h, w), dtype=np.int32)
        label_id = 1

        for r in range(0, h, cell_px):
            for c in range(0, w, cell_px):
                r_end = min(r + cell_px, h)
                c_end = min(c + cell_px, w)
                cell_free = clean[r:r_end, c:c_end] > 0
                if np.sum(cell_free) >= self.min_area:
                    grid_labels[r:r_end, c:c_end][cell_free] = label_id
                    label_id += 1

        valid = list(range(1, label_id))
        return grid_labels, valid, (
            f'{len(valid)} cells via grid fallback '
            f'({self.grid_cell_m:.1f}m cells)'
        )

    # ------------------------------------------------------------------
    def _classify_shape(self, mask: np.ndarray) -> str:
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return 'unknown'
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)
        if perimeter < 1e-3:
            return 'unknown'

        rect = cv2.minAreaRect(cnt)
        w, h = rect[1]
        if min(w, h) < 1e-3:
            return 'unknown'
        aspect = max(w, h) / min(w, h)

        if aspect > 3.0:
            return 'corridor'
        return 'room'

    def _touches_border(self, mask: np.ndarray) -> bool:
        h, w = mask.shape
        m = int(max(1, self.border_margin_px))
        top = mask[:m, :]
        bottom = mask[h - m:, :]
        left = mask[:, :m]
        right = mask[:, w - m:]
        return bool(
            np.any(top > 0) or np.any(bottom > 0) or np.any(left > 0) or np.any(right > 0)
        )


def main(args=None):
    rclpy.init(args=args)
    node = RegionSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
