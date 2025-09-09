#ifdef BNO055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 myIMU = Adafruit_BNO055(-1, 0x28, &Wire);
unsigned long imuPub = 0;  // last time IMU data was published
short IMUReady = 0;

#endif

void initIMU();
void IMU();