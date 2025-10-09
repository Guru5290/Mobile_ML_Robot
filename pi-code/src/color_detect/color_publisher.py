#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class ColourPublisher(Node):
    def __init__(self):
        super().__init__("color_publisher")

        # Publisher to the same topic your service node subscribes to
        self.publisher_ = self.create_publisher(String, "detected_colour", 10)

        # Timer: publish every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_message)
        self.counter = 0

        self.get_logger().info("ColourPublisher started, publishing on /detected_colour")

    def publish_message(self):
        msg = String()

        # Alternate between white and blue for testing
        if self.counter % 2 == 0:
            msg.data = "white"
        else:
            msg.data = "blue"

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = ColourPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
