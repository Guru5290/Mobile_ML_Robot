#ifdef USE_BASE

#ifdef OURENCODER


volatile long revCountFrontL = 0;
volatile long revCountFrontR = 0;
volatile long revCountRearL = 0;
volatile long revCountRearR = 0;

void countFrontLShort() {
  bool a1 = digitalRead(FRONT_LEFT_ENC_PIN_A);
  bool a2 = digitalRead(FRONT_LEFT_ENC_PIN_B);

  if (a2 == LOW) {
    if (a1 == HIGH)
      revCountFrontL++;
    else
      revCountFrontL--;
  } else {
    if (a1 == HIGH)
      revCountFrontL--;
    else
      revCountFrontL++;
  }
}

void countFrontRShort() {
  bool a1 = digitalRead(FRONT_RIGHT_ENC_PIN_A);
  bool a2 = digitalRead(FRONT_RIGHT_ENC_PIN_B);

  if (a2 == LOW) {
    if (a1 == HIGH)
      revCountFrontR++;
    else
      revCountFrontR--;
  } else {
    if (a1 == HIGH)
      revCountFrontR--;
    else
      revCountFrontR++;
  }
}

void countRearLShort() {
  bool a1 = digitalRead(REAR_LEFT_ENC_PIN_A);
  bool a2 = digitalRead(REAR_LEFT_ENC_PIN_B);

  if (a2 == LOW) {
    if (a1 == HIGH)
      revCountRearL++;
    else
      revCountRearL--;
  } else {
    if (a1 == HIGH)
      revCountRearL--;
    else
      revCountRearL++;
  }
}

void countRearRShort() {
  bool a1 = digitalRead(REAR_RIGHT_ENC_PIN_A);
  bool a2 = digitalRead(REAR_RIGHT_ENC_PIN_B);

  if (a2 == LOW) {
    if (a1 == HIGH)
      revCountRearR++;
    else
      revCountRearR--;
  } else {
    if (a1 == HIGH)
      revCountRearR--;
    else
      revCountRearR++;
  }
}

void resetEncoder(int wheel) {

  if (wheel == LEFT) {
    revCountFrontL = 0;
  }

  else if (wheel == RIGHT) {
    revCountFrontR = 0;
  }

  else if (wheel == REAR_LEFT) {
    revCountRearL = 0;
  }

  else if (wheel == REAR_RIGHT) {
    revCountRearR = 0;
  }
}

long readEncoder(int wheel) {
  if (wheel == RIGHT) {
    return revCountFrontR;
  }

  else if (wheel == LEFT) {
    return revCountFrontL;
  }

  else if (wheel == REAR_RIGHT) {
    return revCountRearR;
  }

  else if (wheel == REAR_LEFT) {
    return revCountRearL;
  }
  return 0;
}


#else
#error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
  resetEncoder(REAR_LEFT);
  resetEncoder(REAR_RIGHT);
}

#endif
