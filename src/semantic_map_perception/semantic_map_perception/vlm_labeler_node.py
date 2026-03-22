"""
VLM Labeler Node
Infer semantic room type candidates from region observations using a
Vision-Language Model. Supports OpenAI-compatible API (GPT-4V, local models)
or a configurable endpoint.
"""

import rclpy
from rclpy.node import Node
import base64
import json
import numpy as np

from cv_bridge import CvBridge

from semantic_map_msgs.msg import RegionObservation, VLMResult


class VLMLabelerNode(Node):
    """Queries a VLM with the region's RGB image to classify room type."""

    ROOM_CATEGORIES = [
        'office', 'kitchen', 'bathroom', 'corridor', 'meeting_room',
        'laboratory', 'storage', 'lobby', 'classroom', 'elevator',
        'staircase', 'lounge', 'server_room', 'reception', 'other',
    ]

    def __init__(self):
        super().__init__('vlm_labeler_node')

        self.declare_parameter('api_url', 'http://localhost:11434/api/chat')
        self.declare_parameter('model_name', 'llava')
        self.declare_parameter('api_key', '')
        self.declare_parameter('timeout_sec', 30.0)
        self.declare_parameter('use_openai_format', False)

        self.api_url = self.get_parameter('api_url').value
        self.model_name = self.get_parameter('model_name').value
        self.api_key = self.get_parameter('api_key').value
        self.timeout = self.get_parameter('timeout_sec').value
        self.use_openai = self.get_parameter('use_openai_format').value

        self.bridge = CvBridge()

        self.sub_obs = self.create_subscription(
            RegionObservation, '/semantic_map/region_observation',
            self._on_observation, 10
        )

        self.pub_vlm = self.create_publisher(
            VLMResult, '/semantic_map/vlm_result', 10
        )

        self.get_logger().info(
            f'VLMLabelerNode ready (model={self.model_name}, '
            f'endpoint={self.api_url})'
        )

    def _on_observation(self, msg: RegionObservation):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg.rgb_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image decode error: {e}')
            self._publish_empty(msg.region_id)
            return

        import cv2
        _, buffer = cv2.imencode('.jpg', cv_image)
        b64_image = base64.b64encode(buffer).decode('utf-8')

        prompt = self._build_prompt()

        try:
            labels, scores, rationale = self._query_vlm(b64_image, prompt)
        except Exception as e:
            self.get_logger().error(f'VLM query failed: {e}')
            self._publish_empty(msg.region_id)
            return

        result = VLMResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.region_id = msg.region_id
        result.candidate_labels = labels
        result.candidate_scores = scores
        result.rationale = rationale
        result.valid = len(labels) > 0

        self.pub_vlm.publish(result)
        self.get_logger().info(
            f'Region {msg.region_id}: VLM labels={labels}, scores={scores}'
        )

    def _build_prompt(self) -> str:
        cats = ', '.join(self.ROOM_CATEGORIES)
        return (
            f'You are a room classification assistant for a mobile robot. '
            f'Given this image taken inside a building, classify the room into '
            f'one or more of these categories: [{cats}]. '
            f'Return a JSON object with keys "labels" (list of strings) and '
            f'"scores" (list of floats 0-1) and "rationale" (string). '
            f'Only return valid JSON, nothing else.'
        )

    def _query_vlm(self, b64_image: str, prompt: str):
        import urllib.request
        import urllib.error

        if self.use_openai:
            return self._query_openai(b64_image, prompt)

        payload = {
            'model': self.model_name,
            'messages': [{
                'role': 'user',
                'content': prompt,
                'images': [b64_image],
            }],
            'stream': False,
        }

        data = json.dumps(payload).encode('utf-8')
        req = urllib.request.Request(
            self.api_url,
            data=data,
            headers={'Content-Type': 'application/json'},
        )

        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            body = json.loads(resp.read().decode('utf-8'))

        content = body.get('message', {}).get('content', '{}')
        return self._parse_response(content)

    def _query_openai(self, b64_image: str, prompt: str):
        import urllib.request

        payload = {
            'model': self.model_name,
            'messages': [{
                'role': 'user',
                'content': [
                    {'type': 'text', 'text': prompt},
                    {
                        'type': 'image_url',
                        'image_url': {
                            'url': f'data:image/jpeg;base64,{b64_image}'
                        },
                    },
                ],
            }],
            'max_tokens': 300,
        }

        data = json.dumps(payload).encode('utf-8')
        headers = {
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {self.api_key}',
        }
        req = urllib.request.Request(self.api_url, data=data, headers=headers)

        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            body = json.loads(resp.read().decode('utf-8'))

        content = body['choices'][0]['message']['content']
        return self._parse_response(content)

    def _parse_response(self, content: str):
        content = content.strip()
        if content.startswith('```'):
            lines = content.split('\n')
            content = '\n'.join(lines[1:-1])

        try:
            parsed = json.loads(content)
            labels = parsed.get('labels', [])
            scores = [float(s) for s in parsed.get('scores', [])]
            rationale = parsed.get('rationale', '')

            while len(scores) < len(labels):
                scores.append(0.5)

            return labels[:5], scores[:5], rationale
        except (json.JSONDecodeError, KeyError, TypeError):
            self.get_logger().warn(f'Could not parse VLM response: {content[:200]}')
            return [], [], ''

    def _publish_empty(self, region_id: int):
        result = VLMResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.region_id = region_id
        result.candidate_labels = []
        result.candidate_scores = []
        result.rationale = ''
        result.valid = False
        self.pub_vlm.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = VLMLabelerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
