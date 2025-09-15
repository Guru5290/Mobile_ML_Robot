#include "diffdrive_arduino/diffdrive_arduino.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"



DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
{}


hardware_interface::CallbackReturn DiffDriveArduino::on_init(const hardware_interface::HardwareInfo & info)
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
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Set up the wheels
  front_l_wheel_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev); // ** ** back wheels!
  front_r_wheel_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  back_l_wheel_.setup(cfg_.back_left_wheel_name, cfg_.enc_counts_per_rev); 
  back_r_wheel_.setup(cfg_.back_right_wheel_name, cfg_.enc_counts_per_rev);
  

  // Set up the Arduino
  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);  

  RCLCPP_INFO(logger_, "Finished Configuration");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
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

  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.x", &ax_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.y", &ay_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.z", &az_));

  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.x", &gx_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.y", &gy_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.z", &gz_));

  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.w", &ow_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.x", &ox_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.y", &oy_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.z", &oz_));


  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(front_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &front_l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(front_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &front_r_wheel_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(back_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &back_l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(back_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &back_r_wheel_.cmd));

  return command_interfaces;
}


hardware_interface::CallbackReturn DiffDriveArduino::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  arduino_.sendEmptyMsg();
  // arduino.setPidValues(9,7,0,100);
  // arduino.setPidValues(14,7,0,100);
  // arduino_.setPidValues(30, 20, 0, 100);
  arduino_.setPidValues(200, 120, 1, 50); //currently in use

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduino::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Stopping Controller...");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduino::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // TODO fix chrono duration

  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;


  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  arduino_.readEncoderValues(front_l_wheel_.enc, front_r_wheel_.enc, back_l_wheel_.enc, back_r_wheel_.enc);
  arduino_.readIMUValues(ax_, ay_, az_, gx_, gy_, gz_, ow_, ox_, oy_, oz_);

  double front_pos_prev = front_l_wheel_.pos;
  front_l_wheel_.pos = front_l_wheel_.calcEncAngle();
  front_l_wheel_.vel = (front_l_wheel_.pos - front_pos_prev) / deltaSeconds;

  front_pos_prev = front_r_wheel_.pos;
  front_r_wheel_.pos = front_r_wheel_.calcEncAngle();
  front_r_wheel_.vel = (front_r_wheel_.pos - front_pos_prev) / deltaSeconds;

  double back_pos_prev = back_l_wheel_.pos;
  back_l_wheel_.pos = back_l_wheel_.calcEncAngle();
  back_l_wheel_.vel = (back_l_wheel_.pos - back_pos_prev) / deltaSeconds;

  back_pos_prev = back_r_wheel_.pos;
  back_r_wheel_.pos = back_r_wheel_.calcEncAngle();
  back_r_wheel_.vel = (back_r_wheel_.pos - back_pos_prev) / deltaSeconds;

  return return_type::OK;
 
}

hardware_interface::return_type DiffDriveArduino::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  arduino_.setMotorValues(front_l_wheel_.cmd / front_l_wheel_.rads_per_count / cfg_.loop_rate, front_r_wheel_.cmd / front_r_wheel_.rads_per_count / cfg_.loop_rate, back_l_wheel_.cmd / back_l_wheel_.rads_per_count / cfg_.loop_rate, back_r_wheel_.cmd / back_r_wheel_.rads_per_count / cfg_.loop_rate);

  return return_type::OK;
  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)