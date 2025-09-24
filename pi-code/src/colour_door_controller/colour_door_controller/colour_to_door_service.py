#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray


class ColourToDoorPublisher(Node):
    def __init__(self):
        super().__init__("colour_to_door_publisher")

        # Publisher for servo commands
        self.publisher = self.create_publisher(
            Float64MultiArray, "/servo_controller/commands", 10
        )

        # Subscriber for detected colours
        self.subscription = self.create_subscription(
            String, "detected_colour", self.listener_callback, 10
        )

        self.get_logger().info("Node started: listening to /detected_colour")

    def listener_callback(self, msg: String):
        colour = msg.data.strip().lower()
        self.get_logger().info(f"Received colour: {colour}")

        command = Float64MultiArray()

        if colour == "red":
            command.data = [90.0]
            self.publisher.publish(command)
            self.get_logger().info("Published [90.0] to /servo_controller/commands (OPEN)")
        elif colour == "blue":
            command.data = [170.0]
            self.publisher.publish(command)
            self.get_logger().info("Published [170.0] to /servo_controller/commands (CLOSE)")
        else:
            self.get_logger().warn("Unknown colour received, ignoring")


def main(args=None):
    rclpy.init(args=args)
    node = ColourToDoorPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
