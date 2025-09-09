#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>


struct Config
{
  std::string front_left_wheel_name = "front_left";
  std::string front_right_wheel_name = "front_right";
  std::string back_left_wheel_name = "back_left";
  std::string back_right_wheel_name = "back_right";
  float loop_rate = 30;
  std::string device = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int timeout = 1000;
  int enc_counts_per_rev = 1920;
};


#endif // DIFFDRIVE_ARDUINO_CONFIG_H