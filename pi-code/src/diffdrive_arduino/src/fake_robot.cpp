#include "diffdrive_arduino/fake_robot.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"


FakeRobot::FakeRobot()
  : logger_(rclcpp::get_logger("FakeRobot"))
{}

hardware_interface::CallbackReturn FakeRobot::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.back_left_wheel_name = info_.hardware_parameters["back_left_wheel_name"];
  cfg_.back_right_wheel_name = info_.hardware_parameters["back_right_wheel_name"];


  // Set up the wheels
  // Note: It doesn't matter that we haven't set encoder counts per rev
  // since the fake robot bypasses the encoder code completely

  front_l_wheel_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev); // ** ** back wheels!
  front_r_wheel_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);

  back_l_wheel_.setup(cfg_.back_left_wheel_name, cfg_.enc_counts_per_rev); // ** ** back wheels!
  back_r_wheel_.setup(cfg_.back_right_wheel_name, cfg_.enc_counts_per_rev);

  RCLCPP_INFO(logger_, "Finished Configuration");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FakeRobot::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(front_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &front_l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(front_l_wheel_.name, hardware_interface::HW_IF_POSITION, &front_l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(front_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &front_r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(front_r_wheel_.name, hardware_interface::HW_IF_POSITION, &front_r_wheel_.pos));

  state_interfaces.emplace_back(hardware_interface::StateInterface(back_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &back_l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(back_l_wheel_.name, hardware_interface::HW_IF_POSITION, &back_l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(back_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &back_r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(back_r_wheel_.name, hardware_interface::HW_IF_POSITION, &back_r_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FakeRobot::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(front_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &front_l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(front_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &front_r_wheel_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(back_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &back_l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(back_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &back_r_wheel_.cmd));

  return command_interfaces;
}


hardware_interface::CallbackReturn FakeRobot::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeRobot::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Stopping Controller...");

  return CallbackReturn::SUCCESS;;
}

hardware_interface::return_type FakeRobot::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // TODO fix chrono duration

  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;


  // Force the wheel position
  front_l_wheel_.pos = front_l_wheel_.pos + front_l_wheel_.vel * deltaSeconds;
  front_r_wheel_.pos = front_r_wheel_.pos + front_r_wheel_.vel * deltaSeconds;

  back_l_wheel_.pos = back_l_wheel_.pos + back_l_wheel_.vel * deltaSeconds;
  back_r_wheel_.pos = back_r_wheel_.pos + back_r_wheel_.vel * deltaSeconds;

  return return_type::OK;

  
}

hardware_interface::return_type FakeRobot::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // Set the wheel velocities to directly match what is commanded

  front_l_wheel_.vel = front_l_wheel_.cmd;
  front_r_wheel_.vel = front_r_wheel_.cmd;

  back_l_wheel_.vel = back_l_wheel_.cmd;
  back_r_wheel_.vel = back_r_wheel_.cmd;


  return return_type::OK;  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  FakeRobot,
  hardware_interface::SystemInterface
)