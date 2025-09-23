import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

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

        self.get_logger().info("Press 'd' to capture and publish, 'q' to quit.")

    def spin_loop(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to grab frame")
                break

            cv2.imshow("Potato Leaf Preview", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('d'):
                filename = f"potato_leaf_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                filepath = os.path.join(SAVE_DIR, filename)
                cv2.imwrite(filepath, frame)
                self.get_logger().info(f"Saved image to {filepath}")

                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher_.publish(msg)
                self.get_logger().info("Published captured frame to /image")

            elif key == ord('q'):
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

