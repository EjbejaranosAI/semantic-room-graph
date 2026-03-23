"""
Semantic Fusion Node
Fuse OCR and VLM outputs into a region's semantic state.
Strategy: OCR is prioritized for explicit identifiers (numbers, names).
VLM provides semantic classification. A region becomes 'labeled' when
either valid OCR exists or VLM confidence exceeds the threshold.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from typing import Dict

from semantic_map_msgs.msg import (
    OCRResult, VLMResult, SemanticGraph, RegionNode
)

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class SemanticFusionNode(Node):
    def __init__(self):
        super().__init__('semantic_fusion_node')

        self.declare_parameter('vlm_confidence_threshold', 0.60)
        self.declare_parameter('ocr_priority', True)

        self.vlm_thresh = self.get_parameter('vlm_confidence_threshold').value
        self.ocr_priority = self.get_parameter('ocr_priority').value

        self.graph = None
        self.ocr_buffer: Dict[int, OCRResult] = {}
        self.vlm_buffer: Dict[int, VLMResult] = {}

        self.sub_graph = self.create_subscription(
            SemanticGraph, '/semantic_map/graph_with_poses', self._on_graph, _LATCHED_QOS
        )
        self.sub_ocr = self.create_subscription(
            OCRResult, '/semantic_map/ocr_result', self._on_ocr, 10
        )
        self.sub_vlm = self.create_subscription(
            VLMResult, '/semantic_map/vlm_result', self._on_vlm, 10
        )

        self.pub_semantic_graph = self.create_publisher(
            SemanticGraph, '/semantic_map/semantic_graph', _LATCHED_QOS
        )

        self.get_logger().info(
            f'SemanticFusionNode ready (vlm_threshold={self.vlm_thresh})'
        )

    def _on_graph(self, msg: SemanticGraph):
        self.graph = msg

    def _on_ocr(self, msg: OCRResult):
        self.ocr_buffer[msg.region_id] = msg
        self._try_fuse(msg.region_id)

    def _on_vlm(self, msg: VLMResult):
        self.vlm_buffer[msg.region_id] = msg
        self._try_fuse(msg.region_id)

    def _try_fuse(self, region_id: int):
        if self.graph is None:
            return

        region = None
        for r in self.graph.regions:
            if r.region_id == region_id:
                region = r
                break

        if region is None:
            return

        ocr = self.ocr_buffer.get(region_id)
        vlm = self.vlm_buffer.get(region_id)

        has_ocr = ocr is not None and ocr.valid and len(ocr.detected_text) > 0
        has_vlm = vlm is not None and vlm.valid and len(vlm.candidate_labels) > 0

        if has_ocr:
            region.ocr_text = list(ocr.detected_text)

        if has_ocr and self.ocr_priority:
            region.semantic_label = ' | '.join(ocr.detected_text)
            region.label_confidence = max(ocr.text_confidences) if ocr.text_confidences else 0.8
            region.status = 'labeled'
        elif has_vlm:
            best_idx = 0
            best_score = 0.0
            for i, score in enumerate(vlm.candidate_scores):
                if score > best_score:
                    best_score = score
                    best_idx = i

            if best_score >= self.vlm_thresh:
                region.semantic_label = vlm.candidate_labels[best_idx]
                region.label_confidence = best_score
                region.status = 'labeled'
            else:
                region.semantic_label = vlm.candidate_labels[best_idx] if vlm.candidate_labels else ''
                region.label_confidence = best_score
                region.status = 'partially_labeled'
        else:
            if region.status == 'unexplored':
                region.status = 'explored'

        self.graph.header.stamp = self.get_clock().now().to_msg()
        self.pub_semantic_graph.publish(self.graph)

        self.get_logger().info(
            f'Region {region_id}: status={region.status}, '
            f'label="{region.semantic_label}", '
            f'confidence={region.label_confidence:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SemanticFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
