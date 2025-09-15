#ifndef DIFFDRIVE_ARDUINO_REAL_ROBOT_H
#define DIFFDRIVE_ARDUINO_REAL_ROBOT_H

#include "rclcpp/rclcpp.hpp"

// #include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "config.h"
#include "wheel.h"
#include "arduino_comms.h"

using hardware_interface::return_type;
using hardware_interface::CallbackReturn;

class DiffDriveArduino 
  : public hardware_interface::SystemInterface
{
public:
  DiffDriveArduino();

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  Config cfg_;
  ArduinoComms arduino_;

  Wheel front_l_wheel_;
  Wheel front_r_wheel_;
  Wheel back_l_wheel_;
  Wheel back_r_wheel_;

  double ax_, ay_, az_;
  double gx_, gy_, gz_;
  double ox_, oy_, oz_, ow_;

  rclcpp::Logger logger_{rclcpp::get_logger("DiffDriveArduino")};

  std::chrono::time_point<std::chrono::system_clock> time_;
};

#endif // DIFFDRIVE_ARDUINO_REAL_ROBOT_H
