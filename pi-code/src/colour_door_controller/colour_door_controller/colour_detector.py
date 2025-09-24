#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import numpy as np


class RTSPImagePublisher(Node):
    def __init__(self):
        super().__init__('rtsp_image_publisher')

        # Declare parameter with default value
        self.declare_parameter('IP', '10.122.180.67')
        pi_ip = self.get_parameter('IP').get_parameter_value().string_value

        rtsp_url = f"rtsp://{pi_ip}:8554/cam"
        self.get_logger().info(f"Connecting to {rtsp_url}")

        self.publisher_ = self.create_publisher(String, '/detected_colour', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(rtsp_url)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open RTSP stream")
            rclpy.shutdown()
            return

        self.get_logger().info("Press 'd' to capture and detect, 'q' to quit.")
    
    def detect_colour(self, imageFrame):
        # Convert BGR to HSV 
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

        # ---- RED ---- (two ranges, then combine)
        red_lower1 = np.array([0, 120, 70]) 
        red_upper1 = np.array([10, 255, 255]) 
        red_lower2 = np.array([170, 120, 70]) 
        red_upper2 = np.array([180, 255, 255]) 

        red_mask1 = cv2.inRange(hsvFrame, red_lower1, red_upper1) 
        red_mask2 = cv2.inRange(hsvFrame, red_lower2, red_upper2) 
        red_mask = red_mask1 + red_mask2

        # ---- YELLOW ----
        yellow_lower = np.array([20, 100, 100]) 
        yellow_upper = np.array([30, 255, 255]) 
        yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper) 

        # ---- GREEN ---- (improved range)
        green_lower = np.array([35, 52, 72]) 
        green_upper = np.array([85, 255, 255]) 
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 

        # ---- BLUE ----
        blue_lower = np.array([90, 80, 40]) 
        blue_upper = np.array([130, 255, 255]) 
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper) 

        # Kernel for dilation 
        kernel = np.ones((5, 5), "uint8") 

        # Apply dilations 
        red_mask = cv2.dilate(red_mask, kernel) 
        yellow_mask = cv2.dilate(yellow_mask, kernel) 
        green_mask = cv2.dilate(green_mask, kernel) 
        blue_mask = cv2.dilate(blue_mask, kernel) 

        # ---- Draw contours for each color ----
        area_contours = {"Red":0,"Green":0, "Yellow":0, "Blue":0,  }
    
        def detect_and_draw(mask, color_bgr, label):
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
            for contour in contours: 
                area = cv2.contourArea(contour) 
                if area > 30000: 
                    area_contours[label]=area

                    x, y, w, h = cv2.boundingRect(contour) 
                    cv2.rectangle(imageFrame, (x, y), (x + w, y + h), color_bgr, 2) 
                    cv2.putText(imageFrame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 
                                1.0, color_bgr, 2)
                    
        
        detect_and_draw(red_mask,   (0, 0, 255), "Red")
        detect_and_draw(green_mask, (0, 255, 0), "Green")
        detect_and_draw(yellow_mask, (0, 255, 255), "Yellow")
        detect_and_draw(blue_mask,  (255, 0, 0), "Blue")

        detected_colour = max(area_contours, key=area_contours.get)
        msg = String()
        msg.data = detected_colour
        self.publisher_.publish(msg)
        self.get_logger().info(f"{detected_colour} {area_contours}")


    def spin_loop(self):
        while rclpy.ok():
            ret, imageFrame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to grab frame")
                break

            cv2.imshow("Colour detector preview", imageFrame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('d'):
                self.detect_colour(imageFrame)
                cv2.imshow("Colour detector preview", imageFrame)
                cv2.waitKey(3000) # show preview for 3 seconds

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

