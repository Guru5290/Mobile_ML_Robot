#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class ColourToDoorService(Node):
    def __init__(self):
        super().__init__("colour_to_door_service")

        # Service client
        self.client = self.create_client(SetBool, "door_command")
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Waiting for /door_command service...")

        # Subscriber
        self.subscription = self.create_subscription(
            String, "detected_colour", self.listener_callback, 10
        )

        self.get_logger().info("Node started: listening to /detected_colour")
        self.request = SetBool.Request()

    def listener_callback(self, msg: String):
        colour = msg.data.strip()
        self.get_logger().info(f"Received colour: {colour}")

        if colour.lower() == "red":
            self.get_logger().info("Calling service to OPEN door")
            self.call_service(True)
        elif colour.lower() == "blue":
            self.get_logger().info("Calling service to CLOSE door")
            self.call_service(False)
        else:
            self.get_logger().warn("Unknown colour received, ignoring")

    def call_service(self, state: bool):
        
        self.request.data = state

        if state:
            self.get_logger().info("Calling service to OPEN door (request sent)")
        else:
            self.get_logger().info("Calling service to CLOSE door (request sent)")

        # future = self.client.call_async(self.request)
        return self.client.call_async(self.request)
        # future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            if response is None:
                self.get_logger().error("[CALLBACK] Service returned no response (None)")
                return

            self.get_logger().info(
                f"[CALLBACK] Service response received: success={response.success}, message='{response.message}'"
            )
        except Exception as e:
            self.get_logger().error(f"[CALLBACK] Service call failed with exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ColourToDoorService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
