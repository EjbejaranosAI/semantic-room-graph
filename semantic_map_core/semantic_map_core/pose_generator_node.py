"""
Pose Generator Node
Refine entry_pose for every region so it sits in navigable free space
(maximising distance from walls). Also generates an inspection_pose
slightly offset from the entry.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import cv2
import math

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from semantic_map_msgs.msg import SemanticGraph

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class PoseGeneratorNode(Node):
    def __init__(self):
        super().__init__('pose_generator_node')

        self.declare_parameter('inspection_offset_m', 0.5)
        self.declare_parameter('safety_margin_px', 5)

        self.inspect_offset = self.get_parameter('inspection_offset_m').value
        self.safety_margin = self.get_parameter('safety_margin_px').value

        self.bridge = CvBridge()
        self.grid_image = None
        self.label_image = None
        self.topo_graph = None
        self.map_meta = None

        self.sub_grid = self.create_subscription(
            Image, '/semantic_map/grid_image', self._on_grid, _LATCHED_QOS
        )
        self.sub_label = self.create_subscription(
            Image, '/semantic_map/label_image', self._on_label, _LATCHED_QOS
        )
        self.sub_topo = self.create_subscription(
            SemanticGraph, '/semantic_map/topological_graph',
            self._on_topo, _LATCHED_QOS
        )
        self.sub_meta = self.create_subscription(
            Float32MultiArray, '/semantic_map/map_metadata',
            self._on_meta, _LATCHED_QOS
        )

        self.pub_graph = self.create_publisher(
            SemanticGraph, '/semantic_map/graph_with_poses', _LATCHED_QOS
        )

        self.get_logger().info('PoseGeneratorNode ready.')

    def _on_grid(self, msg: Image):
        self.grid_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self._try_generate()

    def _on_topo(self, msg: SemanticGraph):
        self.topo_graph = msg
        self._try_generate()

    def _on_label(self, msg: Image):
        self.label_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self._try_generate()

    def _on_meta(self, msg: Float32MultiArray):
        self.map_meta = msg.data
        self._try_generate()

    def _try_generate(self):
        if (
            self.grid_image is None
            or self.label_image is None
            or self.topo_graph is None
            or self.map_meta is None
        ):
            return

        graph = self.topo_graph
        binary = self.grid_image
        label_img = self.label_image
        resolution = graph.map_resolution
        origin_x = float(self.map_meta[3])
        origin_y = float(self.map_meta[4])
        h = binary.shape[0]

        dist_map = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
        n_regions = max(len(graph.regions), 1)
        label_step = max(1, 255 // n_regions)

        for idx, region in enumerate(graph.regions):
            entry_wx = region.entry_pose.position.x
            entry_wy = region.entry_pose.position.y

            px = int((entry_wx - origin_x) / resolution) if resolution > 0 else 0
            py = (
                h - 1 - int((entry_wy - origin_y) / resolution)
                if resolution > 0 else 0
            )

            px = np.clip(px, 0, binary.shape[1] - 1)
            py = np.clip(py, 0, binary.shape[0] - 1)

            region_label_value = int((idx + 1) * label_step)
            region_mask = (label_img == region_label_value)

            best_px, best_py = self._find_best_free_pose(
                dist_map, px, py, search_radius=30, allowed_mask=region_mask
            )

            region.entry_pose.position.x = origin_x + best_px * resolution
            region.entry_pose.position.y = origin_y + (h - 1 - best_py) * resolution

            if region.neighbors:
                neighbor_id = region.neighbors[0]
                neighbor = None
                for r in graph.regions:
                    if r.region_id == neighbor_id:
                        neighbor = r
                        break

                if neighbor is not None:
                    dx = neighbor.entry_pose.position.x - region.entry_pose.position.x
                    dy = neighbor.entry_pose.position.y - region.entry_pose.position.y
                    dist = math.hypot(dx, dy)
                    if dist > 1e-3:
                        yaw = math.atan2(dy, dx)
                        region.entry_pose.orientation.z = math.sin(yaw / 2.0)
                        region.entry_pose.orientation.w = math.cos(yaw / 2.0)

            insp_offset_px = int(self.inspect_offset / resolution) if resolution > 0 else 5
            insp_px = np.clip(best_px + insp_offset_px, 0, binary.shape[1] - 1).item()
            insp_py = best_py
            if not region_mask[insp_py, insp_px]:
                insp_px, insp_py = best_px, best_py

            region.inspection_pose.position.x = origin_x + insp_px * resolution
            region.inspection_pose.position.y = origin_y + (h - 1 - insp_py) * resolution
            region.inspection_pose.position.z = 0.0
            region.inspection_pose.orientation.w = 1.0

        graph.header.stamp = self.get_clock().now().to_msg()
        self.pub_graph.publish(graph)
        self.get_logger().info(
            f'Poses generated for {len(graph.regions)} regions'
        )

    def _find_best_free_pose(
        self, dist_map: np.ndarray, cx: int, cy: int, search_radius: int,
        allowed_mask: np.ndarray
    ) -> tuple:
        h, w = dist_map.shape
        traversable_mask = (dist_map > 0.0)
        candidate_mask = traversable_mask & allowed_mask
        if not np.any(candidate_mask):
            best_d = dist_map[cy, cx] if 0 <= cy < h and 0 <= cx < w else 0.0
            return (cx, cy) if best_d > 0.0 else self._nearest_free(dist_map, cx, cy)

        if not candidate_mask[cy, cx]:
            ys, xs = np.where(candidate_mask)
            sq = (xs - cx) ** 2 + (ys - cy) ** 2
            nearest = int(np.argmin(sq))
            cx, cy = int(xs[nearest]), int(ys[nearest])

        best_d = dist_map[cy, cx]
        best = (cx, cy)

        for dy in range(-search_radius, search_radius + 1, 2):
            for dx in range(-search_radius, search_radius + 1, 2):
                ny = cy + dy
                nx = cx + dx
                if 0 <= ny < h and 0 <= nx < w:
                    if not candidate_mask[ny, nx]:
                        continue
                    d = dist_map[ny, nx]
                    if d > best_d:
                        best_d = d
                        best = (nx, ny)

        if best_d <= 0.0:
            ys, xs = np.where(candidate_mask)
            if len(xs) > 0:
                k = int(np.argmax(dist_map[ys, xs]))
                return int(xs[k]), int(ys[k])
        return best

    def _nearest_free(self, dist_map: np.ndarray, cx: int, cy: int) -> tuple:
        ys, xs = np.where(dist_map > 0.0)
        if len(xs) == 0:
            return cx, cy
        sq = (xs - cx) ** 2 + (ys - cy) ** 2
        i = int(np.argmin(sq))
        return int(xs[i]), int(ys[i])


def main(args=None):
    rclpy.init(args=args)
    node = PoseGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
