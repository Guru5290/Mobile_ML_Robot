#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.h>
#include "diffdrive_arduino/diffdrive_arduino.h"
#include "diffdrive_arduino/diffdrive_arduino.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("diffdrive_robot");

  DiffDriveArduino::Config robot_cfg;

  // Attempt to retrieve parameters. If they don't exist, the default values from the struct will be used
  node->get_parameter_or("front_left_wheel_name", robot_cfg.front_left_wheel_name, robot_cfg.front_left_wheel_name);
  node->get_parameter_or("front_right_wheel_name", robot_cfg.front_right_wheel_name, robot_cfg.front_right_wheel_name);
  node->get_parameter_or("back_left_wheel_name", robot_cfg.back_left_wheel_name, robot_cfg.back_left_wheel_name);
  node->get_parameter_or("back_right_wheel_name", robot_cfg.back_right_wheel_name, robot_cfg.back_right_wheel_name);
  node->get_parameter_or("baud_rate", robot_cfg.baud_rate, robot_cfg.baud_rate);
  node->get_parameter_or("device", robot_cfg.device, robot_cfg.device);
  node->get_parameter_or("enc_counts_per_rev", robot_cfg.enc_counts_per_rev, robot_cfg.enc_counts_per_rev);
  node->get_parameter_or("robot_loop_rate", robot_cfg.loop_rate, robot_cfg.loop_rate);

  DiffDriveArduino robot(robot_cfg);
  controller_manager::ControllerManager cm(node);

  rclcpp::Rate loop_rate(10);

  while (rclcpp::ok())
  {
    robot.read();
    cm.update(robot.get_time(), robot.get_period());
    robot.write();

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
}
