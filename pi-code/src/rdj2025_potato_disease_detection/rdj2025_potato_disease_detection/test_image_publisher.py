import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class DismasTheGreat(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        timer_period = 2.0  # publish every 2 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        # Test Image
        cv_image = cv2.imread(
            './src/rdj2025_potato_disease_detection/rdj2025_potato_disease_detection/img_test2.jpg')
        if cv_image is None:
            self.get_logger().error("Could not read image")
            return

        # Publish the image
        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Published test image.')


def main(args=None):
    rclpy.init(args=args)
    node = DismasTheGreat()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




# """
# ROS2 Test Image Publisher Node
# Publishes test images to the /image topic for testing the detection node
# """

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import os
# from pathlib import Path


# class TestImagePublisher(Node):
#     """
#     ROS2 Node that publishes test images for disease detection testing
#     """
    
#     def __init__(self):
#         super().__init__('test_image_publisher')
        
#         # Create publisher for /image topic
#         self.publisher_ = self.create_publisher(Image, '/image', 10)
        
#         # CV Bridge for image conversion
#         self.bridge = CvBridge()
        
#         # Configuration
#         self.declare_parameter('image_path', '')
#         self.declare_parameter('publish_rate', 2.0)  # Hz
#         self.declare_parameter('loop_images', True)
        
#         # Get parameters
#         self.image_path = self.get_parameter('image_path').value
#         publish_rate = self.get_parameter('publish_rate').value
#         self.loop_images = self.get_parameter('loop_images').value
        
#         # Timer for periodic publishing
#         timer_period = 1.0 / publish_rate  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
        
#         # Image list and index for cycling through multiple images
#         self.image_list = []
#         self.current_idx = 0
        
#         # Load images
#         self._load_images()
        
#         self.get_logger().info("="*60)
#         self.get_logger().info("Test Image Publisher Node Started")
#         self.get_logger().info(f"Publishing to: /image")
#         self.get_logger().info(f"Publish rate: {publish_rate} Hz")
#         self.get_logger().info(f"Number of images: {len(self.image_list)}")
#         self.get_logger().info(f"Loop images: {self.loop_images}")
#         self.get_logger().info("="*60)
    
#     def _load_images(self):
#         """
#         Load test images from specified path or default location
#         """
#         # Default image paths to try
#         default_paths = [
#             './src/rdj2025_potato_disease_detection/rdj2025_potato_disease_detection/healthy.jpg',
#             './test_images/Early_blight.jpg',
#             './test_images/Late_blight.jpg',
#             './test_images/Healthy.jpg',
#             './Early_blight.jpg',
#             './Late_blight.jpg',
#             './Healthy.jpg'
#         ]
        
#         # If specific path provided, use it
#         if self.image_path:
#             path = Path(self.image_path)
            
#             # If directory, load all images from it
#             if path.is_dir():
#                 image_extensions = ['.jpg', '.jpeg', '.png', '.JPG', '.JPEG', '.PNG']
#                 for ext in image_extensions:
#                     self.image_list.extend(list(path.glob(f'*{ext}')))
#             # If file, add it
#             elif path.is_file():
#                 self.image_list.append(str(path))
#         else:
#             # Try default paths
#             for default_path in default_paths:
#                 if os.path.exists(default_path):
#                     self.image_list.append(default_path)
        
#         # If no images found, create a test image
#         if not self.image_list:
#             self.get_logger().warning("No test images found. Creating synthetic test image.")
#             self._create_test_image()
#         else:
#             self.get_logger().info(f"Loaded {len(self.image_list)} test image(s):")
#             for img_path in self.image_list:
#                 self.get_logger().info(f"  - {img_path}")
    
#     def _create_test_image(self):
#         """
#         Create a synthetic test image if no real images are available
#         """
#         # Create a simple colored image with text
#         test_image = cv2.imread(cv2.samples.findFile('starry_night.jpg'))
#         if test_image is None:
#             # Create a simple colored rectangle if sample not found
#             test_image = 255 * (0.5 + 0.5 * cv2.randn((480, 640, 3), (0.5, 0.5, 0.5), (0.3, 0.3, 0.3)))
#             test_image = test_image.astype('uint8')
        
#         # Add text
#         cv2.putText(
#             test_image,
#             'Test Image - No real image found',
#             (50, 50),
#             cv2.FONT_HERSHEY_SIMPLEX,
#             1,
#             (255, 255, 255),
#             2,
#             cv2.LINE_AA
#         )
        
#         # Save temporarily
#         temp_path = '/tmp/test_image.jpg'
#         cv2.imwrite(temp_path, test_image)
#         self.image_list.append(temp_path)
    
#     def timer_callback(self):
#         """
#         Callback function that publishes images periodically
#         """
#         if not self.image_list:
#             self.get_logger().error("No images to publish!")
#             return
        
#         # Get current image path
#         image_path = self.image_list[self.current_idx]
        
#         # Read image
#         cv_image = cv2.imread(str(image_path))
        
#         if cv_image is None:
#             self.get_logger().error(f"Could not read image: {image_path}")
#             # Try next image
#             self._advance_index()
#             return
        
#         try:
#             # Convert to ROS Image message
#             msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            
#             # Publish
#             self.publisher_.publish(msg)
            
#             # Log
#             image_name = Path(image_path).name
#             self.get_logger().info(
#                 f'Published image [{self.current_idx + 1}/{len(self.image_list)}]: {image_name}'
#             )
            
#             # Move to next image
#             self._advance_index()
            
#         except Exception as e:
#             self.get_logger().error(f'Error publishing image: {str(e)}')
    
#     def _advance_index(self):
#         """
#         Advance to next image index
#         """
#         if self.loop_images:
#             self.current_idx = (self.current_idx + 1) % len(self.image_list)
#         else:
#             self.current_idx += 1
#             if self.current_idx >= len(self.image_list):
#                 self.get_logger().info("All images published. Stopping...")
#                 self.timer.cancel()


# def main(args=None):
#     """
#     Main function
#     """
#     rclpy.init(args=args)
    
#     try:
#         node = TestImagePublisher()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     except Exception as e:
#         print(f"Error: {e}")
#     finally:
#         if rclpy.ok():
#             node.destroy_node()
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()


