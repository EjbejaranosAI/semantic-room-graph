"""
Topological Graph Builder Node
Creates graph edges (adjacency) from segmented regions by detecting shared
boundaries / narrow transition zones between region pairs.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from semantic_map_msgs.msg import SemanticGraph, RegionEdge

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class TopologicalGraphBuilderNode(Node):
    def __init__(self):
        super().__init__('topological_graph_builder_node')

        self.declare_parameter('adjacency_dilation_px', 3)
        self.declare_parameter('min_transition_width_m', 0.3)

        self.adj_dilation = self.get_parameter('adjacency_dilation_px').value
        self.min_trans_w = self.get_parameter('min_transition_width_m').value

        self.bridge = CvBridge()
        self.label_image = None
        self.regions_raw = None

        self.sub_regions = self.create_subscription(
            SemanticGraph, '/semantic_map/regions_raw', self._on_regions, _LATCHED_QOS
        )
        self.sub_label_img = self.create_subscription(
            Image, '/semantic_map/label_image', self._on_label_image, _LATCHED_QOS
        )

        self.pub_topo_graph = self.create_publisher(
            SemanticGraph, '/semantic_map/topological_graph', _LATCHED_QOS
        )

        self.get_logger().info('TopologicalGraphBuilderNode ready.')

    def _on_label_image(self, msg: Image):
        self.label_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self._try_build()

    def _on_regions(self, msg: SemanticGraph):
        self.regions_raw = msg
        self._try_build()

    def _try_build(self):
        if self.label_image is None or self.regions_raw is None:
            return

        regions = self.regions_raw.regions
        n = len(regions)
        if n < 2:
            out = SemanticGraph()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'map'
            out.regions = list(regions)
            out.edges = []
            out.map_frame = self.regions_raw.map_frame
            out.map_resolution = self.regions_raw.map_resolution
            self.pub_topo_graph.publish(out)
            return

        label_img = self.label_image
        resolution = self.regions_raw.map_resolution

        unique_labels = sorted(set(label_img.flatten()) - {0})
        label_to_region = {}
        for i, lbl in enumerate(unique_labels):
            if i < n:
                label_to_region[lbl] = i

        kernel = np.ones((self.adj_dilation * 2 + 1, self.adj_dilation * 2 + 1), np.uint8)
        edges = []
        neighbor_map = {r.region_id: set() for r in regions}

        for i, lbl_i in enumerate(unique_labels):
            if lbl_i not in label_to_region:
                continue
            rid_i = label_to_region[lbl_i]
            mask_i = (label_img == lbl_i).astype(np.uint8)
            dilated_i = cv2.dilate(mask_i, kernel)

            for j, lbl_j in enumerate(unique_labels):
                if j <= i:
                    continue
                if lbl_j not in label_to_region:
                    continue
                rid_j = label_to_region[lbl_j]
                mask_j = (label_img == lbl_j).astype(np.uint8)

                overlap = cv2.bitwise_and(dilated_i, mask_j)
                overlap_area = int(np.sum(overlap))

                if overlap_area < 1:
                    continue

                ys, xs = np.where(overlap > 0)
                tx = float(np.mean(xs))
                ty = float(np.mean(ys))

                origin_x = self.regions_raw.regions[0].entry_pose.position.x if regions else 0.0
                origin_y = self.regions_raw.regions[0].entry_pose.position.y if regions else 0.0

                transition_width_m = overlap_area * resolution

                edge = RegionEdge()
                edge.header.stamp = self.get_clock().now().to_msg()
                edge.source_region_id = rid_i
                edge.target_region_id = rid_j
                edge.transition_pose.position.x = tx * resolution + origin_x
                edge.transition_pose.position.y = (label_img.shape[0] - ty) * resolution + origin_y
                edge.transition_pose.position.z = 0.0
                edge.transition_pose.orientation.w = 1.0
                edge.transition_width = float(transition_width_m)
                edges.append(edge)

                neighbor_map[rid_i].add(rid_j)
                neighbor_map[rid_j].add(rid_i)

        region_list = list(regions)
        for r in region_list:
            r.neighbors = sorted(neighbor_map.get(r.region_id, set()))

        out = SemanticGraph()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.regions = region_list
        out.edges = edges
        out.map_frame = self.regions_raw.map_frame
        out.map_resolution = self.regions_raw.map_resolution

        self.pub_topo_graph.publish(out)
        self.get_logger().info(
            f'Topological graph built: {len(region_list)} regions, {len(edges)} edges'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TopologicalGraphBuilderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
