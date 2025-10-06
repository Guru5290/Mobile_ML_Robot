#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class RTSPImagePublisher(Node):
    def __init__(self):
        super().__init__('rtsp_image_publisher')

        # Declare parameters
        # self.declare_parameter('IP', '10.122.180.67') #b - Fundi
        self.declare_parameter('IP', '10.226.56.67') #Kabbage - Gareth
        # self.declare_parameter('IP', '192.168.0.112') #Gamefield
        self.declare_parameter('min_area', 1500)
        self.declare_parameter('delay_seconds', 3.0)

        pi_ip = self.get_parameter('IP').get_parameter_value().string_value
        self.min_area = self.get_parameter('min_area').get_parameter_value().integer_value
        self.delay_seconds = self.get_parameter('delay_seconds').get_parameter_value().double_value

        rtsp_url = f"rtsp://{pi_ip}:8554/cam"
        self.get_logger().info(f"Connecting to {rtsp_url}")

        self.publisher_ = self.create_publisher(String, '/detected_colour', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(rtsp_url)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open RTSP stream")
            rclpy.shutdown()
            return

        # State tracking
        self.last_published_colour = None
        self.last_publish_time = 0.0

        self.get_logger().info("Streaming started. Detecting only RED and BLUE.")

    def detect_colour(self, imageFrame):
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        # ---- RED ----
        red_lower1 = np.array([0, 120, 70])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([170, 120, 70])
        red_upper2 = np.array([180, 255, 255])
        red_mask = cv2.inRange(hsvFrame, red_lower1, red_upper1) + cv2.inRange(hsvFrame, red_lower2, red_upper2)
        
        # red_lower = np.array([136, 87, 111], np.uint8) 
        # red_upper = np.array([180, 255, 255], np.uint8) 
        # red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 


        # ---- BLUE ----
        # blue_lower = np.array([90, 80, 40])
        # blue_upper = np.array([130, 255, 255])
        # blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

        # blue_lower = np.array([94, 80, 2], np.uint8) 
        # blue_upper = np.array([120, 255, 255], np.uint8) 
        # blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
        
        blue_lower = np.array([100, 150, 50], np.uint8)
        blue_upper = np.array([130, 255, 255], np.uint8)
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
        
        
        # Kernel for dilation
        kernel = np.ones((5, 5), "uint8")
        red_mask = cv2.dilate(red_mask, kernel)
        blue_mask = cv2.dilate(blue_mask, kernel)
        
        # thresh_preview_window_name = "Red thresh"
        # cv2.namedWindow(thresh_preview_window_name, cv2.WINDOW_NORMAL)
        # cv2.resizeWindow(thresh_preview_window_name, 960, 540)
        # cv2.imshow(thresh_preview_window_name, red_mask)

        # Count contours for each colour
        counts = {"Red": 0, "Blue": 0}

        def count_and_draw(mask, color_bgr, label):
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_area]
            counts[label] = len(valid_contours)
            for contour in valid_contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(imageFrame, (x, y), (x + w, y + h), color_bgr, 2)
                cv2.putText(imageFrame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)

        count_and_draw(red_mask, (0, 0, 255), "Red")
        count_and_draw(blue_mask, (255, 0, 0), "Blue")

        # Decide which colour to publish
        now = time.time()
        if self.last_published_colour is None:
            # First publication: pick colour with most contours
            detected_colour = max(counts, key=counts.get)
            if counts[detected_colour] > 0:
                self._publish_colour(detected_colour, counts)
        else:
            # Already published once, enforce delay
            if (now - self.last_publish_time) >= self.delay_seconds:
                # Only consider the same colour as before
                detected_colour = self.last_published_colour
                if counts[detected_colour] > 3:
                    self._publish_colour(detected_colour, counts)
            # else: still in delay window, ignore

    def _publish_colour(self, colour, counts):
        msg = String()
        msg.data = colour
        self.publisher_.publish(msg)
        self.last_published_colour = colour
        self.last_publish_time = time.time()
        self.get_logger().info(f"Published: {colour} counts={counts}")

    def spin_loop(self):
        window_name = "Colour detector preview"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 960, 540)
        while rclpy.ok():
            ret, imageFrame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to grab frame")
                break

            self.detect_colour(imageFrame)
            cv2.imshow("Colour detector preview", imageFrame)
            key = cv2.waitKey(1) & 0xFF
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
