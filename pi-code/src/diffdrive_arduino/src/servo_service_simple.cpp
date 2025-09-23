#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "arduino_comms.h"  

class ServoServiceNode : public rclcpp::Node
{
public:
  ServoServiceNode()
  : Node("servo_service_node"), comms_("/dev/ttyACM0", 57600, 1000)  
  {
    service_ = this->create_service<std_srvs::srv::SetBool>(
      "door_command",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        if (request->data) {
          RCLCPP_INFO(this->get_logger(), "Sending OPEN command");
          comms_.setServoAngle(0, 90);
          response->message = "Opened";
        } else {
          RCLCPP_INFO(this->get_logger(), "Sending CLOSE command");
          comms_.setServoAngle(0, 170);
          response->message = "Closed";
        }
        response->success = true;
      });

    RCLCPP_INFO(this->get_logger(), "Servo service ready on /door_command");
  }

private:
  ArduinoComms comms_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoServiceNode>());
  rclcpp::shutdown();
  return 0;
}
