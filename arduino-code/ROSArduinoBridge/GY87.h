// disabled to save space

// #include <Adafruit_BMP085.h>
// #include <Wire.h>

// Adafruit_BMP085 bmp;
// short bmpReady = 0;
// unsigned long bmpPub = 0;

// void initBMP();
// void BMP();


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
bool mpuReady = 0;
unsigned long mpuPub = 0;
sensors_event_t a, g, temp;
float mpuReading[3] = { 0, 0, 0 };

void initMPU6050();
void mpu6050();