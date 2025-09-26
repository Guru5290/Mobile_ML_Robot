#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class ColourToDoorService(Node):
    def __init__(self):
        super().__init__("servo_node_ss")

        # Service client
        self.client = self.create_client(SetBool, "door_command")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /door_command service...")

        # Subscriber
        self.subscription = self.create_subscription(
            String, "detected_colour", self.listener_callback, 10
        )

        self.get_logger().info("Node started: listening to /detected_colour")

    def listener_callback(self, msg: String):
        colour = msg.data.strip()
        self.get_logger().info(f"Received colour: {colour}")

        if colour.lower() == "red":
            self.call_service(True)
        elif colour.lower() == "blue":
            self.call_service(False)

    def call_service(self, state: bool):
        request = SetBool.Request()
        request.data = state
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Service response: success={response.success}, message='{response.message}'"
            )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ColourToDoorService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()