#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050_6Axis_MotionApps612.h"  // Better?

#define EARTH_GRAVITY_MS2 9.80665  // m/s2
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

struct MPUDataType_ {
  float Ax = 0;  // Accel
  float Ay = 0;
  float Az = 0;

  float Gx = 0;  // Gyro
  float Gy = 0;
  float Gz = 0;

  float Qx = 0;  // Quaternion
  float Qy = 0;
  float Qz = 0;
  float Qw = 0;
};

bool DMPReady = false;
uint8_t devStatus;
uint8_t FIFOBuffer[64];

MPUDataType_ mpuData;

MPU6050 mpu;
Quaternion q;         // [w, x, y, z]         Quaternion container
VectorInt16 aa;       // [x, y, z]            Accel sensor measurements
VectorInt16 gg;       // [x, y, z]            Gyro sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            World-frame accel sensor measurements
VectorInt16 ggWorld;  // [x, y, z]            World-frame gyro sensor measurements
VectorFloat gravity;  // [x, y, z]            Gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
uint16_t packetSize;  // Expected DMP packet size (default is 42 bytes)


void initMPU6050();
void updateMpu6050();
