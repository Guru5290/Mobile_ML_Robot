#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "arduino_comms.h"  

class ServoServiceNode : public rclcpp::Node {
public:
  ServoServiceNode(const std::string &port, int baud, int timeout_ms)
  : Node("servo_service"), comms_(port, baud, timeout_ms)
  {
    service_ = this->create_service<std_srvs::srv::SetBool>(
      "door_command",
      std::bind(&ServoServiceNode::handleCommand, this,
                std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handleCommand(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data) {
      comms_.setServoAngle(0, 90);
      response->success = true;
      response->message = "Opened";
    } else {
      comms_.setServoAngle(0, 170);
      response->success = true;
      response->message = "Closed";
    }
  }

  ArduinoComms comms_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // default values
  std::string port = "/dev/ttyACM0";
  int baud = 57600;
  int timeout_ms = 1000;

  if (argc > 1) {
    port = argv[1];
  }
  if (argc > 2) {
    baud = std::stoi(argv[2]);
  }
  if (argc > 3) {
    baud = std::stoi(argv[2]);
  }

  auto node = std::make_shared<ServoServiceNode>(port, baud, timeout_ms);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
