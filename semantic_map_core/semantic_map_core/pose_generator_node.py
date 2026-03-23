"""
Pose Generator Node
Generate entry_pose and optional inspection_pose for every region in the
topological graph, ensuring poses are in navigable free space.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import cv2
import math

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from semantic_map_msgs.msg import SemanticGraph


class PoseGeneratorNode(Node):
    def __init__(self):
        super().__init__('pose_generator_node')

        self.declare_parameter('inspection_offset_m', 0.5)
        self.declare_parameter('safety_margin_px', 5)

        self.inspect_offset = self.get_parameter('inspection_offset_m').value
        self.safety_margin = self.get_parameter('safety_margin_px').value

        self.bridge = CvBridge()
        self.grid_image = None
        self.topo_graph = None

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.sub_grid = self.create_subscription(
            Image, '/semantic_map/grid_image', self._on_grid, latched_qos
        )
        self.sub_topo = self.create_subscription(
            SemanticGraph, '/semantic_map/topological_graph', self._on_topo, 10
        )

        self.pub_graph = self.create_publisher(
            SemanticGraph, '/semantic_map/graph_with_poses', 10
        )

        self.get_logger().info('PoseGeneratorNode ready.')

    def _on_grid(self, msg: Image):
        self.grid_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self._try_generate()

    def _on_topo(self, msg: SemanticGraph):
        self.topo_graph = msg
        self._try_generate()

    def _try_generate(self):
        if self.grid_image is None or self.topo_graph is None:
            return

        graph = self.topo_graph
        binary = self.grid_image
        resolution = graph.map_resolution

        dist_map = cv2.distanceTransform(binary, cv2.DIST_L2, 5)

        meta_origin_x = 0.0
        meta_origin_y = 0.0
        if graph.regions:
            meta_origin_x = graph.regions[0].entry_pose.position.x
            meta_origin_y = graph.regions[0].entry_pose.position.y

        for region in graph.regions:
            entry_wx = region.entry_pose.position.x
            entry_wy = region.entry_pose.position.y

            px = int((entry_wx - meta_origin_x) / resolution) if resolution > 0 else 0
            py = binary.shape[0] - int((entry_wy - meta_origin_y) / resolution) if resolution > 0 else 0

            px = np.clip(px, 0, binary.shape[1] - 1)
            py = np.clip(py, 0, binary.shape[0] - 1)

            best_px, best_py = self._find_best_free_pose(
                dist_map, px, py, search_radius=30
            )

            region.entry_pose.position.x = meta_origin_x + best_px * resolution
            region.entry_pose.position.y = meta_origin_y + (binary.shape[0] - best_py) * resolution

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
            insp_px = best_px + insp_offset_px
            insp_py = best_py
            insp_px = np.clip(insp_px, 0, binary.shape[1] - 1)
            insp_py = np.clip(insp_py, 0, binary.shape[0] - 1)

            region.inspection_pose.position.x = meta_origin_x + insp_px * resolution
            region.inspection_pose.position.y = meta_origin_y + (binary.shape[0] - insp_py) * resolution
            region.inspection_pose.position.z = 0.0
            region.inspection_pose.orientation.w = 1.0

        graph.header.stamp = self.get_clock().now().to_msg()
        self.pub_graph.publish(graph)
        self.get_logger().info(
            f'Poses generated for {len(graph.regions)} regions'
        )

    def _find_best_free_pose(
        self, dist_map: np.ndarray, cx: int, cy: int, search_radius: int
    ) -> tuple:
        h, w = dist_map.shape
        best_d = dist_map[cy, cx] if 0 <= cy < h and 0 <= cx < w else 0.0
        best = (cx, cy)

        for dy in range(-search_radius, search_radius + 1, 2):
            for dx in range(-search_radius, search_radius + 1, 2):
                ny = cy + dy
                nx = cx + dx
                if 0 <= ny < h and 0 <= nx < w:
                    d = dist_map[ny, nx]
                    if d > best_d:
                        best_d = d
                        best = (nx, ny)

        return best


def main(args=None):
    rclpy.init(args=args)
    node = PoseGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
