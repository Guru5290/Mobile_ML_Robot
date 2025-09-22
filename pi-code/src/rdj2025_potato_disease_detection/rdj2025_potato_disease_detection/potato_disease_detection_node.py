import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
from .inference_engine import PotatoDiseaseModel
from PIL import Image as PILImage
import cv2

model = PotatoDiseaseModel()

class PotatoDiseaseDetection(Node):
    def __init__(self):
        super().__init__('potato_disease_detection_node')

        # Subscribe to raw image
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.listener_callback,
            10)

        # Publishers
        self.result_pub = self.create_publisher(String, '/inference_result', 10)
        self.annotated_image_pub = self.create_publisher(Image, '/inference_image', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Potato Disease Detection Node has started.")

    def listener_callback(self, msg):
        # Convert ROS Image → OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to PIL for ML model
        pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

        # Run inference
        result = run_inference(pil_image)

        # Publish result as String
        result_msg = String()
        result_msg.data = result
        self.result_pub.publish(result_msg)

        # Overlay result text on image
        annotated_image = cv_image.copy()

        # 🔹 Resize to 1920x1080
        annotated_image = cv2.resize(annotated_image, (1920, 1080))
        h, w, _ = annotated_image.shape

        # Dynamic font scaling
        font_scale = max(1.0, h / 400)
        thickness = max(2, int(h / 300))
        text = f"Result: {result}"

        # --- Automatic line wrapping ---
        words = text.split(" ")
        lines = []
        current_line = ""

        for word in words:
            test_line = current_line + " " + word if current_line else word
            (text_w, _), _ = cv2.getTextSize(test_line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
            if text_w < w - 60:  # leave margin
                current_line = test_line
            else:
                lines.append(current_line)
                current_line = word
        if current_line:
            lines.append(current_line)

        # --- Draw background + text ---
        y0 = 100
        line_height = int(50 * font_scale)

        for i, line in enumerate(lines):
            y = y0 + i * line_height
            if y + line_height > h - 20:
                break

            # Background box (semi-transparent black)
            (text_w, text_h), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
            x, y_baseline = 30, y - text_h
            overlay = annotated_image.copy()
            cv2.rectangle(overlay, (x - 10, y_baseline - 10), (x + text_w + 10, y + 10), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, annotated_image, 0.5, 0, annotated_image)

            # Draw text
            cv2.putText(
                annotated_image,
                line,
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (0, 0, 255),
                thickness,
                cv2.LINE_AA
            )

        # Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        self.annotated_image_pub.publish(annotated_msg)

        self.get_logger().info(f'Published result: {result}')


def run_inference(pil_image) -> str:
    """
    Run actual ML inference using the PotatoDiseaseModel.
    """
    result = model.predict(pil_image)
    print(f"Inference result: {result}")
    return result


def main(args=None):
    rclpy.init(args=args)
    node = PotatoDiseaseDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
