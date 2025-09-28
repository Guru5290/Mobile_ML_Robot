void initMPU6050() {
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment on this line if having compilation difficulties

  mpu.initialize();

  if (mpu.testConnection() == false) {
    // Serial.println("MPU6050 connection failed");
    return;
  }

  devStatus = mpu.dmpInitialize();

  // values obtained from IMU_Zero.ino example sketch
  mpu.setXAccelOffset(-1634);
  mpu.setYAccelOffset(-35);
  // mpu.setZAccelOffset(2687); // on cold start
  mpu.setZAccelOffset(864);  // after many readings
  mpu.setXGyroOffset(25);
  mpu.setYGyroOffset(-90);
  mpu.setZGyroOffset(-15);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    // Serial.println("These are the Active offsets: ");
    // mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);

    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.println(devStatus);
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
}


void updateMpu6050() {
  if (!DMPReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);

    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);

    mpu.dmpGetGyro(&gg, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);

    // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    mpuData.Qw = q.w;
    mpuData.Qx = q.x;
    mpuData.Qy = q.y;
    mpuData.Qz = q.z;

    mpuData.Ax = aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    mpuData.Ay = aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    mpuData.Az = aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;

    mpuData.Gx = ggWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD;
    mpuData.Gy = ggWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD;
    mpuData.Gz = ggWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD;

    // mpuData.yaw = ypr[0] * RAD_TO_DEG;
    // mpuData.pitch = ypr[1] * RAD_TO_DEG;
    // mpuData.roll = ypr[2] * RAD_TO_DEG;
  }
}
