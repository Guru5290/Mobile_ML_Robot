#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>

void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // Needs a variable because setTimeout takes a reference
    serial_conn_.open();
}

void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

void ArduinoComms::readEncoderValues(int &val_1, int &val_2, int &val_3, int &val_4)
{
    std::string response = sendMsg("e\r");

    // Expecting: "enc1 enc2 enc3 enc4"
    std::stringstream ss(response);
    ss >> val_1 >> val_2 >> val_3 >> val_4;
}

void ArduinoComms::setMotorValues(int val_1, int val_2, int val_3, int val_4)
{
    // Send: m motor1 motor2 motor3 motor4
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << " " << val_3 << " " << val_4 << "\r";
    sendMsg(ss.str(), false);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

void ArduinoComms::readIMUValues(double &val_1, double &val_2, double &val_3, double &val_4, double &val_5, double &val_6, double &val_7, double &val_8, double &val_9, double &val_10)
{
    std::string response = sendMsg("I\r");

    // Expecting: Acc.X Acc.Y Acc.Z Gyro.X Gyro.Y Gyro.Z Orient.W Orient.X Orient.Y Orient.Z
    std::stringstream ss(response);
    ss >> val_1 >> val_2 >> val_3 >> val_4 >> val_5 >> val_6 >> val_7 >> val_8 >> val_9 >> val_10;
}

void ArduinoComms::setServoAngle(int servo, int angle)
{
    std::stringstream ss;
    ss << "s " << servo << " " << angle << "\r";
    sendMsg(ss.str());
}

void ArduinoComms::setPumpState(int state)
{
    std::stringstream ss;
    ss << "w " << 33 << " " << state << "\r"; // 33 is PC15
    sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline();

    if (print_output)
    {
        // RCLCPP_INFO_STREAM(logger_, "Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_, "Received: " << response);
    }

    return response;
}
