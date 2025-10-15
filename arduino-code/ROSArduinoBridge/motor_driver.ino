#ifdef USE_BASE

#if defined L298_MOTOR_DRIVER
void initMotorController() {

  pinMode(FRONT_RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(FRONT_RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(FRONT_LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(FRONT_LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(REAR_RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(REAR_RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(REAR_LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(REAR_LEFT_MOTOR_BACKWARD, OUTPUT);

  analogWriteFrequency(30000);
  if (MAX_PWM == 15) analogWriteResolution(4);
  else if (MAX_PWM == 255) analogWriteResolution(8);
  else if (MAX_PWM == 1023) analogWriteResolution(10);
  else if (MAX_PWM == 4095) analogWriteResolution(12);
  else if (MAX_PWM == 65535) analogWriteResolution(16);
}


void setMotorSpeed(int motor, int speed) {

  if (motor == RIGHT) {
    // send 999 or -999 to brake
    // if (abs(speed) == 999) {
    //   digitalWrite(FRONT_RIGHT_MOTOR_FORWARD, HIGH);
    //   digitalWrite(FRONT_RIGHT_MOTOR_BACKWARD, HIGH);
    // }

    // 0 for off
    if (speed == 0) {
      analogWrite(FRONT_RIGHT_MOTOR_FORWARD, 0);
      analogWrite(FRONT_RIGHT_MOTOR_BACKWARD, 0);
    }

    // forward
    else if (speed > 0) {
      analogWrite(FRONT_RIGHT_MOTOR_FORWARD, 0);
      analogWrite(FRONT_RIGHT_MOTOR_BACKWARD, abs(speed) <= MAX_PWM ? abs(speed) : MAX_PWM);
    }

    // reverse
    else {
      analogWrite(FRONT_RIGHT_MOTOR_BACKWARD, 0);
      analogWrite(FRONT_RIGHT_MOTOR_FORWARD, abs(speed) <= MAX_PWM ? abs(speed) : MAX_PWM);
    }
  }

  else if (motor == LEFT) {
    // 0 for off
    if (speed == 0) {
      analogWrite(FRONT_LEFT_MOTOR_FORWARD, 0);
      analogWrite(FRONT_LEFT_MOTOR_BACKWARD, 0);
    }

    // forward
    else if (speed > 0) {
      analogWrite(FRONT_LEFT_MOTOR_FORWARD, 0);
      analogWrite(FRONT_LEFT_MOTOR_BACKWARD, abs(speed) <= MAX_PWM ? abs(speed) : MAX_PWM);
    }

    // reverse
    else {
      analogWrite(FRONT_LEFT_MOTOR_BACKWARD, 0);
      analogWrite(FRONT_LEFT_MOTOR_FORWARD, abs(speed) <= MAX_PWM ? abs(speed) : MAX_PWM);
    }
  }

  else if (motor == REAR_RIGHT) {
    // 0 for off
    if (speed == 0) {
      analogWrite(REAR_RIGHT_MOTOR_FORWARD, 0);
      analogWrite(REAR_RIGHT_MOTOR_BACKWARD, 0);
    }

    // forward
    else if (speed > 0) {
      analogWrite(REAR_RIGHT_MOTOR_FORWARD, 0);
      analogWrite(REAR_RIGHT_MOTOR_BACKWARD, abs(speed) <= MAX_PWM ? abs(speed) : MAX_PWM);
    }

    // reverse
    else {
      analogWrite(REAR_RIGHT_MOTOR_BACKWARD, 0);
      analogWrite(REAR_RIGHT_MOTOR_FORWARD, abs(speed) <= MAX_PWM ? abs(speed) : MAX_PWM);
    }
  }

  else if (motor == REAR_LEFT) {
    // 0 for off
    if (speed == 0) {
      analogWrite(REAR_LEFT_MOTOR_FORWARD, 0);
      analogWrite(REAR_LEFT_MOTOR_BACKWARD, 0);
    }

    // forward
    else if (speed > 0) {
      analogWrite(REAR_LEFT_MOTOR_FORWARD, 0);
      analogWrite(REAR_LEFT_MOTOR_BACKWARD, abs(speed) <= MAX_PWM ? abs(speed) : MAX_PWM);
    }

    // reverse
    else {
      analogWrite(REAR_LEFT_MOTOR_BACKWARD, 0);
      analogWrite(REAR_LEFT_MOTOR_FORWARD, abs(speed) <= MAX_PWM ? abs(speed) : MAX_PWM);
    }
  }
}


void setMotorSpeeds(int leftSpeed, int rightSpeed, int rearLeftSpeed, int rearRightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
  setMotorSpeed(REAR_LEFT, rearLeftSpeed);
  setMotorSpeed(REAR_RIGHT, rearRightSpeed);
}
#else
#error A motor driver must be selected!
#endif

#endif
