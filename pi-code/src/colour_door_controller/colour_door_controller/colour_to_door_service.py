#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import time


class ColourToDoorPublisher(Node):
    def __init__(self):
        super().__init__("colour_to_door_publisher")

        # Declare parameter for delay
        self.declare_parameter('delay_seconds', 30.0)
        self.delay_seconds = self.get_parameter('delay_seconds').get_parameter_value().double_value

        # Publisher for servo commands
        self.publisher = self.create_publisher(Float64MultiArray, "/servo_controller/commands", 10)

        # Subscriber for detected colours
        self.subscription = self.create_subscription(
            String, "detected_colour", self.listener_callback, 10
        )

        # Track last detection times and counts for each color
        self.last_time = {"white": 0.0, "blue": 0.0}
        self.counts = {"white": 0, "blue": 0}

        # Ensure servo starts closed
        self._close_servo()

        self.get_logger().info(
            f"Node started: listening to /detected_colour with delay={self.delay_seconds}s"
        )

    def listener_callback(self, msg: String):
        colour = msg.data.strip().lower()
        now = time.time()

        if colour not in ["white", "blue"]:
            self.get_logger().info(f"Ignoring non-target colour: {colour}")
            return

        # Increment counter only if delay has passed
        if (now - self.last_time[colour]) >= self.delay_seconds:
            self.counts[colour] += 1
            self.last_time[colour] = now
            self.get_logger().info(f"Confirmed {colour.upper()} count={self.counts[colour]}")
        else:
            self.get_logger().info(f"{colour.upper()} detected again too soon, servo remains CLOSED")
            return

        # Check conditions for each color separately
        if self.counts["white"] >= 2:
            self._open_servo("white")
        elif self.counts["blue"] >= 2:
            self._open_servo("blue")
        else:
            self._close_servo()

    def _open_servo(self, colour: str):
        command = Float64MultiArray()
        command.data = [90.0]  # OPEN
        self.publisher.publish(command)

        if colour == "white":
            self.get_logger().info("white confirmed twice -> OPEN (deposit white cube)")
        elif colour == "blue":
            self.get_logger().info("BLUE confirmed twice -> OPEN (deposit blue cube)")

        # Reset after action
        self.counts = {"white": 0, "blue": 0}
        self.last_time = {"white": 0.0, "blue": 0.0}

    def _close_servo(self):
        """Publish a closed command to ensure servo stays closed by default."""
        command = Float64MultiArray()
        command.data = [170.0]  # CLOSED position
        self.publisher.publish(command)
        self.get_logger().info("Servo explicitly set to CLOSED")


def main(args=None):
    rclpy.init(args=args)
    node = ColourToDoorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
