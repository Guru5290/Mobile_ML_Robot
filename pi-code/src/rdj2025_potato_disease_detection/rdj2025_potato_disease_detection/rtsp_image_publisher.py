import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import time

SAVE_DIR = "./src/rdj2025_potato_disease_detection/rdj2025_potato_disease_detection/Captured_Images"  # directory to save images
os.makedirs(SAVE_DIR, exist_ok=True)

class RTSPImagePublisher(Node):
    def __init__(self):
        super().__init__('rtsp_image_publisher')

        # Declare parameter with default value
        self.declare_parameter('IP', '192.168.1.100')
        pi_ip = self.get_parameter('IP').get_parameter_value().string_value

        rtsp_url = f"rtsp://{pi_ip}:8554/cam"
        self.get_logger().info(f"Connecting to {rtsp_url}")

        self.publisher_ = self.create_publisher(Image, '/image', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(rtsp_url)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open RTSP stream")
            rclpy.shutdown()
            return

        # self.get_logger().info("Press 'd' to capture and publish, 'q' to quit.")

    def spin_loop(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to grab frame")
                break

            cv2.imshow("Potato Leaf Preview", frame)
            key = cv2.waitKey(1) & 0xFF

            
            filename = f"potato_leaf_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
            filepath = os.path.join(SAVE_DIR, filename)
            cv2.imwrite(filepath, frame)
            self.get_logger().info(f"Saved image to {filepath}")

            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            # time.sleep(5.0)
            self.publisher_.publish(msg)
            self.get_logger().info("Published captured frame to /image")

            if key == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = RTSPImagePublisher()
    node.spin_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#To run the script with the IP address a parameter:  ros2 run your_package rtsp_image_publisher --ros-args -p pi_ip:=<your_pi_ip>



# """
# ROS2 RTSP Image Publisher Node
# Captures images from RTSP stream and publishes to /image topic
# Compatible with potato disease detection system
# """

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import os
# from datetime import datetime
# from pathlib import Path


# class RTSPImagePublisher(Node):
#     """
#     ROS2 Node that captures and publishes images from RTSP camera stream
#     """
    
#     def __init__(self):
#         super().__init__('rtsp_image_publisher')
        
#         # Declare parameters
#         self.declare_parameter('IP', '192.168.1.100')
#         self.declare_parameter('port', 8554)
#         self.declare_parameter('stream_name', 'cam')
#         self.declare_parameter('continuous_mode', True)  # True for continuous, False for manual
#         self.declare_parameter('publish_rate', 0.2)  # Hz for continuous mode
#         self.declare_parameter('save_images', False)
#         self.declare_parameter('save_dir', './src/rdj2025_potato_disease_detection/rdj2025_potato_disease_detection/Captured_Images')
        
#         # Get parameters
#         pi_ip = self.get_parameter('IP').get_parameter_value().string_value
#         port = self.get_parameter('port').get_parameter_value().integer_value
#         stream_name = self.get_parameter('stream_name').get_parameter_value().string_value
#         self.continuous_mode = self.get_parameter('continuous_mode').get_parameter_value().bool_value
#         publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
#         self.save_images = self.get_parameter('save_images').get_parameter_value().bool_value
#         save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        
#         # Setup save directory
#         self.save_dir = Path(save_dir)
#         if self.save_images:
#             self.save_dir.mkdir(parents=True, exist_ok=True)
#             self.get_logger().info(f"Saving images to: {self.save_dir}")
        
#         # Create publisher
#         self.publisher_ = self.create_publisher(Image, '/image', 10)
        
#         # CV Bridge for image conversion
#         self.bridge = CvBridge()
        
#         # Build RTSP URL
#         self.rtsp_url = f"rtsp://{pi_ip}:{port}/{stream_name}"
#         self.get_logger().info(f"Connecting to RTSP stream: {self.rtsp_url}")
        
#         # Open video capture
#         self.cap = cv2.VideoCapture(self.rtsp_url)
        
#         if not self.cap.isOpened():
#             self.get_logger().error(" Cannot open RTSP stream")
#             self.get_logger().error(f"Check if stream is available at: {self.rtsp_url}")
#             raise RuntimeError("Failed to open RTSP stream")
        
#         # Get stream properties
#         self.stream_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
#         self.stream_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
#         self.stream_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
#         self.get_logger().info("="*60)
#         self.get_logger().info("RTSP Image Publisher Started")
#         self.get_logger().info(f"Stream Resolution: {self.stream_width}x{self.stream_height}")
#         self.get_logger().info(f"Stream FPS: {self.stream_fps}")
#         self.get_logger().info(f"Publishing to: /image")
#         self.get_logger().info(f"Mode: {'Continuous' if self.continuous_mode else 'Manual Capture'}")
        
#         if self.continuous_mode:
#             self.get_logger().info(f"Publish Rate: {publish_rate} Hz")
#             self.get_logger().info("Press 'q' in preview window to quit")
#         else:
#             self.get_logger().info("Press 'd' to capture & publish, 'q' to quit")
        
#         self.get_logger().info("="*60)
        
#         # Timer for continuous mode
#         self.timer = None
#         if self.continuous_mode:
#             timer_period = 1.0 / publish_rate
#             self.timer = self.create_timer(timer_period, self.timer_callback)
        
#         # Statistics
#         self.frame_count = 0
#         self.published_count = 0
#         self.saved_count = 0
    
#     def timer_callback(self):
#         """
#         Timer callback for continuous publishing mode
#         """
#         ret, frame = self.cap.read()
        
#         if not ret:
#             self.get_logger().error("Failed to grab frame from stream")
#             return
        
#         self.frame_count += 1
        
#         # Publish frame
#         self._publish_frame(frame)
        
#         # Optionally save frame
#         if self.save_images and self.frame_count % 10 == 0:  # Save every 10th frame
#             self._save_frame(frame)
    
#     def _publish_frame(self, frame):
#         """
#         Publish frame to /image topic
#         """
#         try:
#             msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
#             self.publisher_.publish(msg)
#             self.published_count += 1
            
#             if self.published_count % 10 == 0:
#                 self.get_logger().info(f"Published {self.published_count} frames")
                
#         except Exception as e:
#             self.get_logger().error(f"Error publishing frame: {str(e)}")
    
#     def _save_frame(self, frame):
#         """
#         Save frame to disk
#         """
#         try:
#             timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
#             filename = f"potato_leaf_{timestamp}.jpg"
#             filepath = self.save_dir / filename
            
#             cv2.imwrite(str(filepath), frame)
#             self.saved_count += 1
#             self.get_logger().info(f"✓ Saved image: {filename}")
            
#         except Exception as e:
#             self.get_logger().error(f"Error saving frame: {str(e)}")
    
#     def spin_loop(self):
#         """
#         Main loop for manual capture mode (keyboard control)
#         """
#         self.get_logger().info("Starting manual capture mode...")
        
#         try:
#             while rclpy.ok():
#                 ret, frame = self.cap.read()
                
#                 if not ret:
#                     self.get_logger().error("Failed to grab frame from stream")
#                     break
                
#                 self.frame_count += 1
                
#                 # Show preview window
#                 cv2.imshow("Potato Leaf Preview - Press 'd' to capture", frame)
                
#                 # Check for keyboard input
#                 key = cv2.waitKey(1) & 0xFF
                
#                 if key == ord('d'):
#                     # Capture and publish
#                     self.get_logger().info("📸 Capturing image...")
                    
#                     # Save image
#                     if self.save_images:
#                         self._save_frame(frame)
                    
#                     # Publish to ROS topic
#                     self._publish_frame(frame)
#                     self.get_logger().info("✓ Frame captured and published to /image")
                    
#                 elif key == ord('q'):
#                     self.get_logger().info("Quit command received")
#                     break
                
#                 elif key == ord('c'):
#                     # Toggle continuous mode
#                     self.continuous_mode = not self.continuous_mode
#                     mode_str = "Continuous" if self.continuous_mode else "Manual"
#                     self.get_logger().info(f"Switched to {mode_str} mode")
                    
#                     if self.continuous_mode and self.timer is None:
#                         # Start timer
#                         self.timer = self.create_timer(0.5, self.timer_callback)
#                     elif not self.continuous_mode and self.timer is not None:
#                         # Stop timer
#                         self.timer.cancel()
#                         self.timer = None
                
#                 # Process ROS callbacks
#                 rclpy.spin_once(self, timeout_sec=0)
                
#         except KeyboardInterrupt:
#             self.get_logger().info("Interrupted by user")
        
#         finally:
#             # Cleanup
#             self._cleanup()
    
#     def _cleanup(self):
#         """
#         Cleanup resources
#         """
#         self.get_logger().info("Cleaning up...")
        
#         # Print statistics
#         self.get_logger().info("="*60)
#         self.get_logger().info("Session Statistics:")
#         self.get_logger().info(f"  Total frames processed: {self.frame_count}")
#         self.get_logger().info(f"  Frames published: {self.published_count}")
#         self.get_logger().info(f"  Images saved: {self.saved_count}")
#         self.get_logger().info("="*60)
        
#         # Release resources
#         if self.cap is not None:
#             self.cap.release()
#         cv2.destroyAllWindows()
        
#         self.get_logger().info("Cleanup complete")
    
#     def __del__(self):
#         """
#         Destructor - ensure cleanup
#         """
#         if hasattr(self, 'cap') and self.cap is not None:
#             self.cap.release()
#         cv2.destroyAllWindows()


# def main(args=None):
#     """
#     Main function to run the RTSP publisher node
#     """
#     rclpy.init(args=args)
    
#     try:
#         node = RTSPImagePublisher()
        
#         # Use spin_loop for manual mode, regular spin for continuous mode
#         if node.continuous_mode:
#             # Continuous mode - use ROS spin with preview window
#             import threading
            
#             def show_preview():
#                 """Show preview in separate thread"""
#                 while rclpy.ok():
#                     ret, frame = node.cap.read()
#                     if ret:
#                         cv2.imshow("Potato Leaf Stream (Continuous)", frame)
#                         if cv2.waitKey(1) & 0xFF == ord('q'):
#                             break
#                 cv2.destroyAllWindows()
            
#             # Start preview thread
#             preview_thread = threading.Thread(target=show_preview, daemon=True)
#             preview_thread.start()
            
#             # Spin ROS node
#             rclpy.spin(node)
#         else:
#             # Manual mode - use custom spin loop
#             node.spin_loop()
            
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



# """
# 1. Basic usage (default IP):
#    ros2 run rdj2025_potato_disease_detection rtsp_publisher

# 2. Custom IP address:
#    ros2 run rdj2025_potato_disease_detection rtsp_publisher --ros-args -p IP:=192.168.1.150

# 3. Continuous publishing mode (auto-publish at 0.2 Hz):
#    ros2 run rdj2025_potato_disease_detection rtsp_publisher --ros-args -p continuous_mode:=true -p publish_rate:=0.2

# 4. Custom RTSP configuration:
#    ros2 run rdj2025_potato_disease_detection rtsp_publisher --ros-args \
#      -p IP:=192.168.1.150 \
#      -p port:=8554 \
#      -p stream_name:=cam

# 5. Disable image saving:
#    ros2 run rdj2025_potato_disease_detection rtsp_publisher --ros-args -p save_images:=false

# 6. Custom save directory:
#    ros2 run rdj2025_potato_disease_detection rtsp_publisher --ros-args \
#      -p save_dir:=/home/user/my_images

# KEYBOARD CONTROLS (Manual Mode):
# - Press 'd' to capture and publish frame
# - Press 'c' to toggle continuous mode on/off
# - Press 'q' to quit

# CONTINUOUS MODE:
# - Automatically publishes frames at specified rate
# - Still shows preview window
# - Press 'q' in preview window to quit
# """