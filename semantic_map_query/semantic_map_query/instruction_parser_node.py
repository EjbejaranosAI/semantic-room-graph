"""
Instruction Parser Node
Receive the user instruction (text) and extract deterministic cues such as
numbers, keywords, or room names. Publishes a structured QueryRequest.
"""

import re
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from semantic_map_msgs.msg import QueryRequest


class InstructionParserNode(Node):
    def __init__(self):
        super().__init__('instruction_parser_node')

        self.sub_query = self.create_subscription(
            String, '/semantic_map/user_query', self._on_query, 10
        )

        self.pub_request = self.create_publisher(
            QueryRequest, '/semantic_map/query_request', 10
        )

        self.get_logger().info('InstructionParserNode ready.')

    def _on_query(self, msg: String):
        raw = msg.data.strip()
        if not raw:
            return

        request = QueryRequest()
        request.header.stamp = self.get_clock().now().to_msg()
        request.raw_query = raw

        numbers = re.findall(r'\b\d+\b', raw)
        if numbers:
            request.has_numeric_reference = True
            request.numeric_reference = int(numbers[0])
        else:
            request.has_numeric_reference = False
            request.numeric_reference = -1

        self.pub_request.publish(request)
        self.get_logger().info(
            f'Parsed query: "{raw}" '
            f'(numeric={request.has_numeric_reference}, '
            f'ref={request.numeric_reference})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = InstructionParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
