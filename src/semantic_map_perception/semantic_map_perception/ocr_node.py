"""
OCR Node
Extract textual evidence from region observations using EasyOCR.
Detects door numbers, room names, signs, etc.
"""

import rclpy
from rclpy.node import Node
import numpy as np

from cv_bridge import CvBridge

from semantic_map_msgs.msg import RegionObservation, OCRResult


class OCRNode(Node):
    def __init__(self):
        super().__init__('ocr_node')

        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('languages', ['en', 'es'])

        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.languages = self.get_parameter('languages').value

        self.bridge = CvBridge()
        self.reader = None

        self.sub_obs = self.create_subscription(
            RegionObservation, '/semantic_map/region_observation',
            self._on_observation, 10
        )

        self.pub_ocr = self.create_publisher(
            OCRResult, '/semantic_map/ocr_result', 10
        )

        self.get_logger().info('OCRNode ready (lazy-loading EasyOCR on first call).')

    def _ensure_reader(self):
        if self.reader is not None:
            return True
        try:
            import easyocr
            self.reader = easyocr.Reader(self.languages, gpu=False)
            self.get_logger().info('EasyOCR reader initialized.')
            return True
        except ImportError:
            self.get_logger().error(
                'easyocr not installed. Run: pip install easyocr'
            )
            return False

    def _on_observation(self, msg: RegionObservation):
        if not self._ensure_reader():
            self._publish_empty(msg.region_id)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg.rgb_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to decode image: {e}')
            self._publish_empty(msg.region_id)
            return

        try:
            results = self.reader.readtext(cv_image)
        except Exception as e:
            self.get_logger().error(f'OCR failed: {e}')
            self._publish_empty(msg.region_id)
            return

        detected_text = []
        confidences = []
        for (_bbox, text, conf) in results:
            if conf >= self.conf_thresh:
                detected_text.append(text.strip())
                confidences.append(float(conf))

        ocr_result = OCRResult()
        ocr_result.header.stamp = self.get_clock().now().to_msg()
        ocr_result.region_id = msg.region_id
        ocr_result.detected_text = detected_text
        ocr_result.text_confidences = confidences
        ocr_result.valid = len(detected_text) > 0

        self.pub_ocr.publish(ocr_result)
        self.get_logger().info(
            f'Region {msg.region_id}: OCR found {len(detected_text)} texts: {detected_text}'
        )

    def _publish_empty(self, region_id: int):
        ocr_result = OCRResult()
        ocr_result.header.stamp = self.get_clock().now().to_msg()
        ocr_result.region_id = region_id
        ocr_result.detected_text = []
        ocr_result.text_confidences = []
        ocr_result.valid = False
        self.pub_ocr.publish(ocr_result)


def main(args=None):
    rclpy.init(args=args)
    node = OCRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
