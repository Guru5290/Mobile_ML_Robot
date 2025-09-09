// void initIMU() {
//   IMUReady = myIMU.begin();

//   if (!IMUReady) {
//     mySerial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//     return;
//   }

//   int8_t temp = myIMU.getTemp();
//   mySerial.print("IMU Sensor Ready! Temp: ");
//   mySerial.println(temp);
//   myIMU.setExtCrystalUse(true);
// }

// void IMU() {


//   if (millis() < imuPub + SENSOR_SAMPLERATE_DELAY_MS) {
//     return;
//   }
//   imuPub = millis();

//   if (!IMUReady) {
//     mySerial.println("No BNO055 detected");
//     return;
//   }


//   //Serial.println("Getting IMU DATA .... ");
//   imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//   // imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//   // imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

//   mySerial.print("Acceleration: ");
//   mySerial.print(acc.x());
//   mySerial.print(" , ");
//   mySerial.print(acc.y());
//   mySerial.print(" , ");
//   mySerial.print(acc.z());
//   mySerial.println("");

//   //Serial.print(mag.x());
//   //Serial.print(",");
//   //Serial.print(mag.y());
//   //Serial.print(",");
//   //Serial.print(mag.z());
//   //Serial.print(",");

//   // delay(BNO055_SAMPLERATE_DELAY_MS);
//   // delay(100);
// }
