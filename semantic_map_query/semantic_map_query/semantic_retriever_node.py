"""
Semantic Retriever Node
Two-stage query: (1) deterministic lookup over OCR text and labels,
(2) semantic fallback using an LLM to generate ranked candidates that
are projected onto the known semantic space of the graph.
"""

import json
import rclpy
from rclpy.node import Node
from difflib import SequenceMatcher

from semantic_map_msgs.msg import (
    QueryRequest, QueryResult, QueryCandidate, SemanticGraph
)


class SemanticRetrieverNode(Node):
    def __init__(self):
        super().__init__('semantic_retriever_node')

        self.declare_parameter('similarity_threshold', 0.5)
        self.declare_parameter('llm_api_url', 'http://localhost:11434/api/generate')
        self.declare_parameter('llm_model', 'llama3')
        self.declare_parameter('use_llm_fallback', True)

        self.sim_thresh = self.get_parameter('similarity_threshold').value
        self.llm_url = self.get_parameter('llm_api_url').value
        self.llm_model = self.get_parameter('llm_model').value
        self.use_llm = self.get_parameter('use_llm_fallback').value

        self.graph = None

        self.sub_graph = self.create_subscription(
            SemanticGraph, '/semantic_map/semantic_graph', self._on_graph, 10
        )
        self.sub_request = self.create_subscription(
            QueryRequest, '/semantic_map/query_request', self._on_request, 10
        )

        self.pub_result = self.create_publisher(
            QueryResult, '/semantic_map/query_candidates', 10
        )

        self.get_logger().info('SemanticRetrieverNode ready.')

    def _on_graph(self, msg: SemanticGraph):
        self.graph = msg

    def _on_request(self, msg: QueryRequest):
        if self.graph is None:
            self.get_logger().warn('No graph available, cannot process query.')
            return

        candidates = self._deterministic_lookup(msg)

        if not candidates and self.use_llm:
            candidates = self._semantic_fallback(msg)

        result = QueryResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.raw_query = msg.raw_query
        result.candidates = candidates

        if candidates:
            best = max(candidates, key=lambda c: c.final_score)
            result.selected_region_id = best.region_id
            region = self._find_region(best.region_id)
            if region:
                result.selected_goal_pose = region.entry_pose
            result.success = True
            result.message = f'Matched region {best.region_id} ({best.matched_label})'
        else:
            result.selected_region_id = -1
            result.success = False
            result.message = 'No matching region found'

        self.pub_result.publish(result)
        self.get_logger().info(
            f'Query "{msg.raw_query}" -> {result.message}'
        )

    def _deterministic_lookup(self, request: QueryRequest) -> list:
        candidates = []

        for region in self.graph.regions:
            if region.status not in ('labeled', 'partially_labeled'):
                continue

            score = 0.0
            matched_label = ''

            if request.has_numeric_reference:
                num_str = str(request.numeric_reference)
                for ocr_text in region.ocr_text:
                    if num_str in ocr_text:
                        score = 1.0
                        matched_label = ocr_text
                        break

            if score < 1.0:
                query_lower = request.raw_query.lower()
                label_lower = region.semantic_label.lower()
                sim = SequenceMatcher(None, query_lower, label_lower).ratio()
                if sim > self.sim_thresh and sim > score:
                    score = sim
                    matched_label = region.semantic_label

                for ocr_text in region.ocr_text:
                    sim_ocr = SequenceMatcher(
                        None, query_lower, ocr_text.lower()
                    ).ratio()
                    if sim_ocr > self.sim_thresh and sim_ocr > score:
                        score = sim_ocr
                        matched_label = ocr_text

            if score > 0.0:
                c = QueryCandidate()
                c.region_id = region.region_id
                c.matched_label = matched_label
                c.semantic_score = float(score)
                c.region_confidence = region.label_confidence
                c.navigation_cost = 0.0
                c.final_score = float(score * region.label_confidence)
                candidates.append(c)

        candidates.sort(key=lambda c: c.final_score, reverse=True)
        return candidates[:5]

    def _semantic_fallback(self, request: QueryRequest) -> list:
        known_labels = set()
        for r in self.graph.regions:
            if r.semantic_label:
                known_labels.add(r.semantic_label.lower())

        if not known_labels:
            return []

        prompt = (
            f'A user asks a robot to go to: "{request.raw_query}". '
            f'The robot knows these room labels: {sorted(known_labels)}. '
            f'Which label(s) best match the request? '
            f'Return a JSON list of objects with "label" and "score" (0-1). '
            f'Only valid JSON, no explanation.'
        )

        try:
            llm_candidates = self._query_llm(prompt)
        except Exception as e:
            self.get_logger().warn(f'LLM fallback failed: {e}')
            return []

        candidates = []
        for lc in llm_candidates:
            label = lc.get('label', '').lower()
            score = float(lc.get('score', 0.5))

            for region in self.graph.regions:
                if region.semantic_label.lower() == label:
                    c = QueryCandidate()
                    c.region_id = region.region_id
                    c.matched_label = region.semantic_label
                    c.semantic_score = score
                    c.region_confidence = region.label_confidence
                    c.navigation_cost = 0.0
                    c.final_score = score * region.label_confidence
                    candidates.append(c)

        candidates.sort(key=lambda c: c.final_score, reverse=True)
        return candidates[:5]

    def _query_llm(self, prompt: str) -> list:
        import urllib.request

        payload = {
            'model': self.llm_model,
            'prompt': prompt,
            'stream': False,
        }

        data = json.dumps(payload).encode('utf-8')
        req = urllib.request.Request(
            self.llm_url,
            data=data,
            headers={'Content-Type': 'application/json'},
        )

        with urllib.request.urlopen(req, timeout=30) as resp:
            body = json.loads(resp.read().decode('utf-8'))

        content = body.get('response', '[]')
        content = content.strip()
        if content.startswith('```'):
            lines = content.split('\n')
            content = '\n'.join(lines[1:-1])

        return json.loads(content)

    def _find_region(self, region_id: int):
        if self.graph is None:
            return None
        for r in self.graph.regions:
            if r.region_id == region_id:
                return r
        return None


def main(args=None):
    rclpy.init(args=args)
    node = SemanticRetrieverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
