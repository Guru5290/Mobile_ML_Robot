#define ADDR1_TOF 0x28
#define ADDR2_TOF 0x29

void initTOF() {
  mySerial.println("Adafruit VL53L0X test.");

  for (int i = 0; i < 4; i++) {  // try both addresses till something sticks
    TOFReady = lox.begin(0x28);
    delay(50);
    if (TOFReady)
      break;
    mySerial.println("TOF addr 0x28 failed");

    TOFReady = lox.begin(0x29);
    delay(50);
    if (TOFReady)
      break;
    mySerial.println("TOF addr 0x29 failed");
  }


  if (!TOFReady) {
    mySerial.println("Failed to boot VL53L0X");
    return;
  }

  // power
  mySerial.println("VL53L0X API Simple Ranging example\n\n");
}

void TOF() {

  if (millis() < tofPub + SENSOR_SAMPLERATE_DELAY_MS) {
    return;
  }
  tofPub = millis();

  if (!TOFReady) {
    mySerial.println("No GYL530x detected");
    return;
  }

  lox.rangingTest(&measure, false);  // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {    // phase failures have incorrect data
    tofReading = measure.RangeMilliMeter;
  } else {
    tofReading = 9999;
  }
}
