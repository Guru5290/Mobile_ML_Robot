#include "Arduino.h"
// #ifdef BNO055
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

bool TOFReady = 0;
unsigned long tofPub = 0;  // last time TOF data was published
unsigned int tofReading = 0;
VL53L0X_RangingMeasurementData_t measure;

// #endif

void initTOF();
void TOF();
