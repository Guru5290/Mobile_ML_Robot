// disabled to save space
// void initBMP() {
//   bmpReady = bmp.begin();
//   if (!bmpReady) {
//     mySerial.println("Could not find a valid BMP085 sensor, check wiring!");
//   }
// }

// void BMP() {

//   if (millis() < bmpPub + SENSOR_SAMPLERATE_DELAY_MS) {
//     return;
//   }
//   bmpPub = millis();

//   if (!bmpReady) {
//     mySerial.println("No BMP detected...");
//     return;
//   }

//   mySerial.print("Temperature = ");
//   mySerial.print(bmp.readTemperature());
//   mySerial.print(" *C");

//   mySerial.print(" Pressure = ");
//   mySerial.print(bmp.readPressure());
//   mySerial.println(" Pa");
// }

void initMPU6050(void) {
  mySerial.println("Adafruit MPU6050 test!");
  mpuReady = mpu.begin();

  // Try to initialize!
  if (!mpuReady) {
    mySerial.println("Failed to find MPU6050 chip");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void mpu6050() {

  if (millis() < mpuPub + SENSOR_SAMPLERATE_DELAY_MS) {
    return;
  }
  mpuPub = millis();

  if (!mpuReady) {
    mySerial.println("No MPU6050 detected");
    return;
  }

  mpu.getEvent(&a, &g, &temp);

  mpuReading[0] = a.acceleration.x;
  mpuReading[1] = a.acceleration.y;
  mpuReading[2] = a.acceleration.z;

  // mySerial.print(("Acc X: "));
  // mySerial.print(a.acceleration.x);
  // mySerial.print((", Y: "));
  // mySerial.print(a.acceleration.y);
  // mySerial.print((", Z: "));
  // mySerial.print(a.acceleration.z);
  // mySerial.println((" m/s^2"));

  // mySerial.print(" Rotation X: ");
  // mySerial.print(g.gyro.x);
  // mySerial.print(", Y: ");
  // mySerial.print(g.gyro.y);
  // mySerial.print(", Z: ");
  // mySerial.print(g.gyro.z);
  // mySerial.print(" rad/s");

  // mySerial.print(" Temperature: ");
  // mySerial.print(temp.temperature);
  // mySerial.println(" degC");
}
