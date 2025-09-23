#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ServoServiceNode : public rclcpp::Node
{
public:
  ServoServiceNode() : Node("servo_service_node")
  {
    service_ = this->create_service<std_srvs::srv::SetBool>(
      "door_command",
      std::bind(&ServoServiceNode::handle_service, this,
                std::placeholders::_1, std::placeholders::_2));

    servo_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/servo_controller/commands", 10);

    RCLCPP_INFO(this->get_logger(), "Servo service ready!");
  }

private:
  void handle_service(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std_msgs::msg::Float64MultiArray cmd;
    // double angle_radians;
    cmd.data.resize(1); // one servo channel
    if (request->data) {
      // cmd.data[0] = angle_radians; 
      cmd.data[0] = 90.0; 
      response->message = "Door opened";
      
    } else {
      // cmd.data[0] = angle_radians; 
      cmd.data[0] = 170.0; 
      response->message = "Door closed";
    }

    servo_pub_->publish(cmd);

    response->success = true;
  }

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
