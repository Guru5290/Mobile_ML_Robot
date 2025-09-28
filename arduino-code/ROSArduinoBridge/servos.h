#ifndef SERVOS_H
#define SERVOS_H

#include <Servo.h>

#define N_SERVOS 2

// This delay in milliseconds determines the pause
// between each one degree step the servo travels.  Increasing
// this number will make the servo sweep more slowly.
// Decreasing this number will make the servo sweep more quickly.
// Zero is the default number and will make the servos spin at
// full speed.  150 ms makes them spin very slowly.
int stepDelay[N_SERVOS] = { 10, 10 };  // ms

// Pins
byte servoPins[N_SERVOS] = { PB10, PA5 };  // T2C1, T2C3

// Initial Position
byte servoInitPosition[N_SERVOS] = { 170, 170 };  // [0, 180] degrees


class SweepServo {
public:
  SweepServo();
  void initServo(
    int servoPin,
    int stepDelayMs,
    int initPosition);
  void doSweep();
  void setTargetPosition(int position);
  Servo getServo();
private:
  Servo servo;
  int stepDelayMs;
  int currentPositionDegrees;
  int targetPositionDegrees;
  long lastSweepCommand;
};

SweepServo servos[N_SERVOS];

#endif
