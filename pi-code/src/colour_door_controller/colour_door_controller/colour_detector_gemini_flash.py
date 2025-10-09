# gemini flash 
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

        # Declare parameters (Keep your existing parameter declarations)
        
        # self.declare_parameter('IP', '10.178.118.67') #b - Fundi
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
            # In a real ROS node, you should ideally handle this more gracefully,
            # perhaps with a timer to retry the connection.
            # For now, let's keep the shutdown:
            rclpy.shutdown()
            return

        # State tracking
        self.last_published_colour = None
        self.last_publish_time = 0.0

        self.get_logger().info("Streaming started. Detecting and publishing dominant colour.")

    # --- New Helper Function for Contour Processing ---
    def _get_max_contour_area_and_draw(self, mask, imageFrame, color_bgr, label):
        """Finds max contour area for a mask, draws valid contours, and returns max area."""
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_area:
                if area > max_area:
                    max_area = area
                
                # Draw the valid contour
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(imageFrame, (x, y), (x + w, y + h), color_bgr, 2)
                cv2.putText(imageFrame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
        
        return max_area
    # ----------------------------------------------------

    def detect_colour(self, imageFrame):
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        # 1. Define Colour Masks
        
        # ---- RED ---- (Re-enabled the standard split-hue range for Red)
        red_lower1 = np.array([0, 120, 70], np.uint8)
        red_upper1 = np.array([10, 255, 255], np.uint8)
        red_lower2 = np.array([170, 120, 70], np.uint8)
        red_upper2 = np.array([180, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower1, red_upper1) + cv2.inRange(hsvFrame, red_lower2, red_upper2)

        # ---- BLUE ---- (Keep your existing blue range)
        blue_lower = np.array([100, 150, 50], np.uint8)
        blue_upper = np.array([130, 255, 255], np.uint8)
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
        
        # ---- WHITE ---- (Keep your existing white range, which targets low Saturation, high Value)
        white_lower = np.array([0, 0, 110], np.uint8)
        white_upper = np.array([179, 70, 255], np.uint8)
        white_mask = cv2.inRange(hsvFrame, white_lower, white_upper)
        
        # 2. Apply Dilation (Keep your existing kernel for dilation)
        kernel = np.ones((5, 5), "uint8")
        red_mask = cv2.dilate(red_mask, kernel) # Dilation applied
        blue_mask = cv2.dilate(blue_mask, kernel) # Dilation applied
        white_mask = cv2.dilate(white_mask, kernel) # Dilation applied

        # (Optional: Keep your threshold preview window if needed)
        # thresh_preview_window_name = "white thresh"
        # cv2.namedWindow(thresh_preview_window_name, cv2.WINDOW_NORMAL)
        # cv2.resizeWindow(thresh_preview_window_name, 960, 540)
        # cv2.imshow(thresh_preview_window_name, white_mask)

        # 3. Process Masks and Find Max Area
        max_areas = {}
        
        # Use the new helper function to calculate the max contour area and draw
        max_areas["Red"] = self._get_max_contour_area_and_draw(red_mask, imageFrame, (0, 0, 255), "Red")
        max_areas["White"] = self._get_max_contour_area_and_draw(white_mask, imageFrame, (255, 255, 255), "White")
        max_areas["Blue"] = self._get_max_contour_area_and_draw(blue_mask, imageFrame, (255, 0, 0), "Blue")
        
        # 4. Determine Dominant Colour
        # Find the colour with the maximum area.
        # Use a list of colours to prioritize if areas are equal (optional, but good practice)
        
        dominant_colour = "None"
        max_area_val = 0
        
        # Check all detected colours and find the absolute largest area
        for color, area in max_areas.items():
            if area > max_area_val:
                max_area_val = area
                dominant_colour = color

        # 5. Publish with Delay Enforcement
        now = time.time()
        
        # The logic is simpler now: only publish if a colour was detected AND the delay has passed.
        # We don't need to check for a minimum count, as we already filtered by min_area.
        
        # Check if a colour was actually detected (max_area_val > 0) AND the delay has passed
        if max_area_val > 0 and (self.last_published_colour is None or (now - self.last_publish_time) >= self.delay_seconds):
            self._publish_colour(dominant_colour, max_area_val)
        
        # Optional: You could add logic here to *keep* publishing the previously published
        # colour if it's still the dominant one and the delay is active, but a simple
        # "publish when delay is over" is generally safer for state changes.

    def _publish_colour(self, colour, max_area):
        msg = String()
        msg.data = colour
        self.publisher_.publish(msg)
        self.last_published_colour = colour
        self.last_publish_time = time.time()
        self.get_logger().info(f"Published: {colour} (Max Area: {max_area:.0f})")

    # The spin_loop and main functions remain the same.
    def spin_loop(self):
        window_name = "Colour detector preview"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 960, 540)
        while rclpy.ok():
            ret, imageFrame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to grab frame")
                # Try to reconnect or wait before trying again
                time.sleep(1.0)
                self.cap = cv2.VideoCapture(f"rtsp://{self.get_parameter('IP').get_parameter_value().string_value}:8554/cam")
                if not self.cap.isOpened():
                    self.get_logger().error("Still cannot open RTSP stream. Breaking loop.")
                    break
                continue

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