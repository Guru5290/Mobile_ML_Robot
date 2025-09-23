#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "arduino_comms.h"  

class ServoServiceNode : public rclcpp::Node
{
public:
  ServoServiceNode()
  : Node("servo_service")
  {
    this->declare_parameter<std::string>("device", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 57600);
    this->declare_parameter<int>("timeout_ms", 1000);

    std::string port = this->get_parameter("device").as_string();
    int baud = this->get_parameter("baud_rate").as_int();
    int timeout_ms = this->get_parameter("timeout_ms").as_int();

    RCLCPP_INFO(this->get_logger(), "Connecting to %s @ %d baud %d timeout", port.c_str(), baud, timeout_ms);

    comms_ = std::make_shared<ArduinoComms>(port, baud, timeout_ms);

    service_ = this->create_service<std_srvs::srv::SetBool>(
      "door_command",
      std::bind(&ServoServiceNode::handle_command, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "Servo service ready on /door_command");
  }

private:
  void handle_command(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data) {
      comms_->setServoAngle(0, 90);
      response->success = true;
      response->message = "Door opened";
    } else {
      comms_->setServoAngle(0, 170);
      response->success = true;
      response->message = "Door closed";
    }
  }

  std::shared_ptr<ArduinoComms> comms_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
