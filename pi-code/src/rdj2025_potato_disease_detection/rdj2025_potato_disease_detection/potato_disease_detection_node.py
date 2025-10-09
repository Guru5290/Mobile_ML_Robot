# from collections import deque
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import cv2
# from PIL import Image as PILImage
# from .inference_engine import PotatoDiseaseModel

# model = PotatoDiseaseModel()

# # Globals
# n_times = deque(maxlen=5)
# finalized_result = None
# finalized_annotated_image = None   # <-- store the locked image here
# result = None


# class PotatoDiseaseDetection(Node):
#     def __init__(self):
#         super().__init__('potato_disease_detection_node')

#         # Subscribe to raw image
#         self.subscription = self.create_subscription(
#             Image,
#             '/image',
#             self.listener_callback,
#             10)

#         # Publishers
#         self.result_pub = self.create_publisher(String, '/inference_result', 10)
#         self.annotated_image_pub = self.create_publisher(Image, '/inference_image', 10)

#         timer_period = 5.0
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.bridge = CvBridge()

#         self.get_logger().info("Potato Disease Detection Node has started.")

#     def listener_callback(self, msg):
#         global result, finalized_result, finalized_annotated_image

#         # If result already finalized → stop further inference
#         if finalized_result is not None:
#             return

#         # Convert ROS Image → OpenCV image
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

#         # Run inference
#         result = run_inference(pil_image)

#         # Overlay result text on the image
#         annotated_image = cv_image.copy()
#         annotated_image = cv2.resize(annotated_image, (1920, 1080))
#         h, w, _ = annotated_image.shape

#         font_scale = max(1.0, h / 400)
#         thickness = max(2, int(h / 300))
#         text = f"Result: {result}"

#         # Draw simple background box
#         (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
#         overlay = annotated_image.copy()
#         cv2.rectangle(overlay, (30, 80), (30 + text_w + 20, 80 + text_h + 40), (0, 0, 0), -1)
#         cv2.addWeighted(overlay, 0.5, annotated_image, 0.5, 0, annotated_image)
#         cv2.putText(
#             annotated_image,
#             text,
#             (40, 120),
#             cv2.FONT_HERSHEY_SIMPLEX,
#             font_scale,
#             (0, 0, 255),
#             thickness,
#             cv2.LINE_AA
#         )

#         # Publish result immediately (before finalization)
#         result_msg = String()
#         result_msg.data = result
#         self.result_pub.publish(result_msg)

#         # Save this annotated image if we finalize
#         if finalized_result is None and is_finalized():
#             finalized_annotated_image = annotated_image.copy()

#         # Keep last annotated_image in memory (used before locking)
#         self.last_annotated_image = annotated_image

#     def timer_callback(self):
#         global finalized_result, finalized_annotated_image

#         # Choose correct image: finalized one if locked, otherwise the most recent
#         if finalized_result is not None and finalized_annotated_image is not None:
#             image_to_publish = finalized_annotated_image
#         else:
#             if not hasattr(self, "last_annotated_image"):
#                 return
#             image_to_publish = self.last_annotated_image

#         # Publish annotated image
#         annotated_msg = self.bridge.cv2_to_imgmsg(image_to_publish, encoding='bgr8')
#         self.annotated_image_pub.publish(annotated_msg)

#         # If finalized, publish stable result continuously
#         if finalized_result is not None:
#             result_msg = String()
#             result_msg.data = finalized_result
#             self.result_pub.publish(result_msg)
#             self.get_logger().info(f' Finalized result: {finalized_result}')


# def run_inference(pil_image) -> str:
#     """
#     Run actual ML inference using the PotatoDiseaseModel.
#     """
#     global result, n_times, finalized_result

#     if finalized_result is not None:
#         return finalized_result  # Skip further inference

#     result = model.predict(pil_image)
#     print(f"Inference result: {result}")

#     n_times.append(result)
#     print(f"History: {list(n_times)}")

#     # Check stabilization
#     if len(n_times) == 5:
#         for label in set(n_times):
#             if n_times.count(label) > 3:
#                 finalized_result = label
#                 print(f"Finalized result locked: {finalized_result}")
#                 break

#     return result


# def is_finalized() -> bool:
#     """Helper to check if the global finalized_result has been locked."""
#     global finalized_result
#     return finalized_result is not None


# def main(args=None):
#     rclpy.init(args=args)
#     node = PotatoDiseaseDetection()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()






"""
ROS2 Node for Potato Disease Detection  <<<   THIS SCRIPT SHOWS CONFIDENCE LEVELS BUT HAS NO TIMERCALLBACK INSTEAD THE PUBLISHER PUBLISHES AFTER A  VARIABLE RATE
Subscribes to image topic and publishes inference results
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
from .inference_engine import PotatoDiseaseModel
from PIL import Image as PILImage
import cv2


class PotatoDiseaseDetection(Node):
    """
    ROS2 Node for real-time potato disease detection
    """
    
    def __init__(self):
        super().__init__('potato_disease_detection_node')
        
        # Initialize the ML model (singleton pattern)
        self.get_logger().info("Loading potato disease detection model...")
        self.model = PotatoDiseaseModel()
        self.get_logger().info("Model loaded successfully!")
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribe to raw image topic
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.listener_callback,
            10
        )
        
        # Publisher for inference result (text)
        self.result_pub = self.create_publisher(
            String, 
            '/inference_result', 
            10
        )
        
        # Publisher for annotated image with result overlay
        self.annotated_image_pub = self.create_publisher(
            Image, 
            '/inference_image', 
            10
        )
        
        # Configuration
        self.output_width = 1920
        self.output_height = 1080
        
        self.get_logger().info("="*60)
        self.get_logger().info("Potato Disease Detection Node has started")
        self.get_logger().info(f"Subscribed to: /image")
        self.get_logger().info(f"Publishing to: /inference_result, /inference_image")
        self.get_logger().info(f"Output resolution: {self.output_width}x{self.output_height}")
        self.get_logger().info("="*60)
    
    def listener_callback(self, msg):
        """
        Callback function for image messages
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert BGR to RGB and then to PIL Image for the model
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            
            # Run inference
            prediction, confidences = self.model.predict_with_confidence(pil_image)
            
            # Create result message
            result_msg = String()
            result_msg.data = prediction
            self.result_pub.publish(result_msg)
            
            # Create annotated image
            annotated_image = self._create_annotated_image(
                cv_image, 
                prediction, 
                confidences
            )
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(
                annotated_image, 
                encoding='bgr8'
            )
            self.annotated_image_pub.publish(annotated_msg)
            
            # Log result
            confidence_pct = confidences[prediction] * 100
            self.get_logger().info(
                f'Detected: {prediction} ({confidence_pct:.1f}% confidence)'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in callback: {str(e)}')
    
    def _create_annotated_image(self, cv_image, prediction, confidences):
        """
        Create annotated image with prediction overlay
        """
        # Resize to target output resolution
        annotated = cv2.resize(cv_image, (self.output_width, self.output_height))
        h, w, _ = annotated.shape
        
        # Dynamic font scaling based on image size
        font_scale = max(1.0, h / 400)
        thickness = max(2, int(h / 300))
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # Get confidence for the predicted class
        confidence_pct = confidences[prediction] * 100
        
        # Determine color based on prediction
        color_map = {
            'Healthy': (0, 255, 0),        # Green
            'Early_blight': (0, 165, 255),  # Orange
            'Late_blight': (0, 0, 255)      # Red
        }
        text_color = color_map.get(prediction, (255, 255, 255))
        
        # Prepare text lines
        lines = [
            f"Status: {prediction}",
            f"Confidence: {confidence_pct:.1f}%"
        ]
        
        # Add all class probabilities
        lines.append("")
        lines.append("All Classes:")
        for class_name in sorted(confidences.keys()):
            conf = confidences[class_name] * 100
            lines.append(f"  {class_name}: {conf:.1f}%")
        
        # Calculate text dimensions and positions
        y0 = 50
        line_height = int(45 * font_scale)
        padding = 20
        
        # Draw each line with background
        for i, line in enumerate(lines):
            y = y0 + i * line_height
            
            # Skip if out of bounds
            if y + line_height > h - 20:
                break
            
            # Get text size
            (text_w, text_h), baseline = cv2.getTextSize(
                line, font, font_scale, thickness
            )
            
            # Position
            x = padding
            y_text = y + text_h
            
            # Draw semi-transparent background
            overlay = annotated.copy()
            cv2.rectangle(
                overlay,
                (x - 10, y - 5),
                (x + text_w + 10, y + text_h + 10),
                (0, 0, 0),
                -1
            )
            cv2.addWeighted(overlay, 0.6, annotated, 0.4, 0, annotated)
            
            # Draw text
            cv2.putText(
                annotated,
                line,
                (x, y_text),
                font,
                font_scale,
                text_color,
                thickness,
                cv2.LINE_AA
            )
        
        # Add status indicator box
        indicator_size = 30
        indicator_x = w - indicator_size - 20
        indicator_y = 20
        
        cv2.rectangle(
            annotated,
            (indicator_x, indicator_y),
            (indicator_x + indicator_size, indicator_y + indicator_size),
            text_color,
            -1
        )
        cv2.rectangle(
            annotated,
            (indicator_x, indicator_y),
            (indicator_x + indicator_size, indicator_y + indicator_size),
            (255, 255, 255),
            2
        )
        
        return annotated


def main(args=None):
    """
    Main function to run the detection node
    """
    rclpy.init(args=args)
    
    try:
        node = PotatoDiseaseDetection()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()