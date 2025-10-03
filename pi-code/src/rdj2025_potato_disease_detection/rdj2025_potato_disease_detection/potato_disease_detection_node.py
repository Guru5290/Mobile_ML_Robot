from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage
from .inference_engine import PotatoDiseaseModel

model = PotatoDiseaseModel()

# Globals
n_times = deque(maxlen=5)
finalized_result = None
finalized_annotated_image = None   # <-- store the locked image here
result = None


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

        timer_period = 5.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()

        self.get_logger().info("Potato Disease Detection Node has started.")

    def listener_callback(self, msg):
        global result, finalized_result, finalized_annotated_image

        # If result already finalized → stop further inference
        if finalized_result is not None:
            return

        # Convert ROS Image → OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

        # Run inference
        result = run_inference(pil_image)

        # Overlay result text on the image
        annotated_image = cv_image.copy()
        annotated_image = cv2.resize(annotated_image, (1920, 1080))
        h, w, _ = annotated_image.shape

        font_scale = max(1.0, h / 400)
        thickness = max(2, int(h / 300))
        text = f"Result: {result}"

        # Draw simple background box
        (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        overlay = annotated_image.copy()
        cv2.rectangle(overlay, (30, 80), (30 + text_w + 20, 80 + text_h + 40), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, annotated_image, 0.5, 0, annotated_image)
        cv2.putText(
            annotated_image,
            text,
            (40, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 0, 255),
            thickness,
            cv2.LINE_AA
        )

        # Publish result immediately (before finalization)
        result_msg = String()
        result_msg.data = result
        self.result_pub.publish(result_msg)

        # Save this annotated image if we finalize
        if finalized_result is None and is_finalized():
            finalized_annotated_image = annotated_image.copy()

        # Keep last annotated_image in memory (used before locking)
        self.last_annotated_image = annotated_image

    def timer_callback(self):
        global finalized_result, finalized_annotated_image

        # Choose correct image: finalized one if locked, otherwise the most recent
        if finalized_result is not None and finalized_annotated_image is not None:
            image_to_publish = finalized_annotated_image
        else:
            if not hasattr(self, "last_annotated_image"):
                return
            image_to_publish = self.last_annotated_image

        # Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(image_to_publish, encoding='bgr8')
        self.annotated_image_pub.publish(annotated_msg)

        # If finalized, publish stable result continuously
        if finalized_result is not None:
            result_msg = String()
            result_msg.data = finalized_result
            self.result_pub.publish(result_msg)
            self.get_logger().info(f' Finalized result: {finalized_result}')


def run_inference(pil_image) -> str:
    """
    Run actual ML inference using the PotatoDiseaseModel.
    """
    global result, n_times, finalized_result

    if finalized_result is not None:
        return finalized_result  # Skip further inference

    result = model.predict(pil_image)
    print(f"Inference result: {result}")

    n_times.append(result)
    print(f"History: {list(n_times)}")

    # Check stabilization
    if len(n_times) == 5:
        for label in set(n_times):
            if n_times.count(label) > 3:
                finalized_result = label
                print(f"Finalized result locked: {finalized_result}")
                break

    return result


def is_finalized() -> bool:
    """Helper to check if the global finalized_result has been locked."""
    global finalized_result
    return finalized_result is not None


def main(args=None):
    rclpy.init(args=args)
    node = PotatoDiseaseDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
