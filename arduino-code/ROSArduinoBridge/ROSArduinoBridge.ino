// 988 cpr

// python3 -m serial.tools.miniterm /dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Mega_2560_12148509806232150650-if00 57600 --echo
// python3 -m serial.tools.miniterm /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 57600 --echo
// "/home/l/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/bin/avrdude" "-C/home/l/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/etc/avrdude.conf" -v -V -patmega2560 -cwiring "-P/dev/ttyACM0" -b115200 -D "-Uflash:w:/tmp/arduino/sketches/71D686EBAB249562A9FB6FE352DD37F9/ROSArduinoBridge.ino.hex:i"

/*
board - generic stm32f4 series
board part number - blackpill F401CC
Upload method - STM32Cube DFU
USB support generic CDC serial

Ensure no IO and no pin definitions on USB D+ and D- pins
*/

// issues
// occasional spikes on left wheel on new m value, seems when pid is not in use and motors were moved (perhaps by hand) the spikes occur, done?
// loading then releasing front left causes rear left to also spike, done?
// time delay before motors start moving for small values of m
// investigate the occasional spikes, done?
// investigate the occasional divergence on small values of m
// m 0 -50

// proposed changes
// add prevenc = enc in diff_controller when not moving
// add enc = readEnc in forFixingStartupSpikes in rosarduinobridge

#define USE_BASE  // Enable the base controller code
// #undef USE_BASE     // Disable the base controller code

#ifdef USE_BASE

#define OURENCODER

#define L298_MOTOR_DRIVER

// #define tuning_pid rearLeftPID  // comment this line to disable tuning mode

#endif

#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
// #undef USE_SERVOS  // Disable use of PWM servos

#define BAUDRATE 57600

// #define MAX_PWM 255
#define MAX_PWM 1023
// #define MAX_PWM 4095

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "commands.h"
#include "sensors.h"

#ifdef USE_SERVOS
#include "servos.h"
#endif

#ifdef USE_BASE

#include "motor_driver.h"
#include "encoder_driver.h"
#include "diff_controller.h"
#include "imu_driver.h"


#define PID_RATE 30  // Hz
// #define PID_RATE 50  // Hz
#define SENSOR_SAMPLERATE_DELAY_MS 30

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int inde_x = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];
char argv3[16];
char argv4[16];

// The arguments converted to integers
long arg1;
long arg2;
long arg3;
long arg4;


/* Clear the current command parameters */
void resetCommand() {
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg4 = 0;

  arg = 0;
  inde_x = 0;
}

/* Run a command.  Commands are defined in commands.h */
void runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];

  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);
  arg4 = atoi(argv4);

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0)
        digitalWrite(arg1, LOW);
      else if (arg2 == 1)
        digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    case PIN_MODE:
      if (arg2 == 0)
        pinMode(arg1, INPUT);
      else if (arg2 == 1)
        pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    case PING:
      Serial.println(Ping(arg1));
      break;
#ifdef USE_SERVOS
    case SERVO_WRITE:
      servos[arg1].setTargetPosition(arg2);
      Serial.println("OK");
      break;
    case SERVO_READ:
      Serial.println(servos[arg1].getServo().read());
      break;
#endif

#ifdef USE_BASE
    case READ_ENCODERS:
      Serial.printf("%ld %ld %ld %ld\n", readEncoder(LEFT), readEncoder(RIGHT), readEncoder(REAR_LEFT), readEncoder(REAR_RIGHT));
      // Serial.printf("%ld %ld\n", readEncoder(LEFT), readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      // Serial.printf("Setting motor speeds to %ld %ld %ld %ld\n", arg1, arg2, arg3, arg4);
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0 && arg3 == 0 && arg4 == 0) {
        setMotorSpeeds(0, 0, 0, 0);
        resetPID();
        moving = 0;
      } else
        moving = 1;

      forFixingStartupKick();

      leftPID.TargetTicksPerFrame = arg1;
      rightPID.TargetTicksPerFrame = arg2;
      rearLeftPID.TargetTicksPerFrame = arg3;
      rearRightPID.TargetTicksPerFrame = arg4;

      Serial.println("OK");
      break;

    case MOTOR_RAW_PWM:
      // Serial.printf("Setting motor speeds to %ld %ld %ld %ld\n", arg1, arg2, arg3, arg4);
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      resetPID();
      moving = 0;  // Sneaky way to temporarily disable the PID
      setMotorSpeeds(arg1, arg2, arg3, arg4);
      Serial.println("OK");
      break;

    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != nullptr) {
        pid_args[i] = atoi(str);
        i++;
      }

      Kp = pid_args[0];
      Kd = pid_args[1];
      Ki = pid_args[2];
      Ko = pid_args[3];
      Serial.println("OK");
      break;

    case GET_CURRENT_PID:
      Serial.printf("%d %d %d %d\n", Kp, Kd, Ki, Ko);
      break;

    case GO_FORWARD:
      lastMotorCommand = millis();
      setMotorSpeedsFeedback(60, 60, 60, 60);
      Serial.println("OK");
      break;

    case GO_BACKWARD:
      lastMotorCommand = millis();
      setMotorSpeedsFeedback(-60, -60, -60, -60);
      Serial.println("OK");
      break;

    case TURN_LEFT:
      lastMotorCommand = millis();
      setMotorSpeedsFeedback(-30, 30, -30, 30);
      Serial.println("OK");
      break;

    case TURN_RIGHT:
      lastMotorCommand = millis();
      setMotorSpeedsFeedback(30, -30, 30, -30);
      Serial.println("OK");
      break;

#endif
    case READ_IMU:
      // ax ay az gx gy gz
      Serial.print(mpuData.Ax);
      Serial.print(" ");
      Serial.print(mpuData.Ay);
      Serial.print(" ");
      Serial.print(mpuData.Az);
      Serial.print(" ");
      Serial.print(mpuData.Gx);
      Serial.print(" ");
      Serial.print(mpuData.Gy);
      Serial.print(" ");
      Serial.print(mpuData.Gz);
      Serial.print(" ");
      Serial.print(mpuData.Qw);
      Serial.print(" ");
      Serial.print(mpuData.Qx);
      Serial.print(" ");
      Serial.print(mpuData.Qy);
      Serial.print(" ");
      Serial.print(mpuData.Qz);
      Serial.println();
      break;
    default:
      Serial.println("Invalid Command Serial");
      break;
  }
}


/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

  // for detecting dfu mode
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW);

// Initialize the motor controller if used */
#ifdef USE_BASE

#ifdef OURENCODER

  pinMode(FRONT_LEFT_ENC_PIN_A, INPUT);
  pinMode(FRONT_LEFT_ENC_PIN_B, INPUT);
  pinMode(FRONT_RIGHT_ENC_PIN_A, INPUT);
  pinMode(FRONT_RIGHT_ENC_PIN_B, INPUT);
  pinMode(REAR_LEFT_ENC_PIN_A, INPUT);
  pinMode(REAR_LEFT_ENC_PIN_B, INPUT);
  pinMode(REAR_RIGHT_ENC_PIN_A, INPUT);
  pinMode(REAR_RIGHT_ENC_PIN_B, INPUT);

  attachInterrupt(FRONT_LEFT_ENC_PIN_A, countFrontLShort, CHANGE);
  attachInterrupt(FRONT_RIGHT_ENC_PIN_A, countFrontRShort, CHANGE);
  attachInterrupt(REAR_LEFT_ENC_PIN_A, countRearLShort, CHANGE);
  attachInterrupt(REAR_RIGHT_ENC_PIN_A, countRearRShort, CHANGE);

#endif
  initMotorController();
  resetPID();
#endif

  /* Attach servos if used */
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].initServo(
      servoPins[i],
      stepDelay[i],
      servoInitPosition[i]);
  }
#endif

  initMPU6050();
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {

  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // if (chr==10 || chr == 13) {  // terminate with \n or \r
    if (chr == 13) {  // terminate with \r
      if (arg == 1)
        argv1[inde_x] = '\0';
      else if (arg == 2)
        argv2[inde_x] = '\0';
      else if (arg == 3)
        argv3[inde_x] = '\0';
      else if (arg == 4)
        argv4[inde_x] = '\0';

      runCommand();
      resetCommand();

      while (Serial.available()) {  // to clear Serial buffer
        Serial.read();
      }
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0)
        arg = 1;
      else if (arg == 1) {
        argv1[inde_x] = '\0';
        arg = 2;
        inde_x = 0;
      } else if (arg == 2) {
        argv2[inde_x] = '\0';
        arg = 3;
        inde_x = 0;
      } else if (arg == 3) {
        argv3[inde_x] = '\0';
        arg = 4;
        inde_x = 0;
      }
      continue;
    } else {

      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      } else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[inde_x] = chr;
        inde_x++;
      } else if (arg == 2) {
        argv2[inde_x] = chr;
        inde_x++;
      } else if (arg == 3) {
        argv3[inde_x] = chr;
        inde_x++;
      } else if (arg == 4) {
        argv4[inde_x] = chr;
        inde_x++;
      }
    }
  }

// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

#ifndef tuning_pid
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0, 0, 0);
    moving = 0;
  }
#endif

#endif

  static unsigned long mpuPub = 0;
  if (millis() - mpuPub > SENSOR_SAMPLERATE_DELAY_MS) {
    mpuPub = millis();
    updateMpu6050();
  }

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif

#ifdef tuning_pid
  forTuningPID();
  if (tuning_pid.TargetTicksPerFrame != 0)
    Serial.printf("%ld\n", tuning_pid.Encoder);
    // Serial.printf("%ld %d %d 0 80 \n", output, input, (int)tuning_pid.TargetTicksPerFrame);
    // Serial.printf("%ld %ld I %ld %ld %d %ld\n", Kp * Perror, Kd * (input - tuning_pid.PrevInput), tuning_pid.ITerm, output, (int)tuning_pid.TargetTicksPerFrame, input);
    // Pterm Dterm Iterm PWMval Target Actual
#endif

  // static unsigned long lastPrinted = 0;
  // if (millis() - lastPrinted > 20) {
  //   lastPrinted = millis();
  //   Serial.printf("%ld %ld %ld %ld\n", readEncoder(LEFT), readEncoder(RIGHT), readEncoder(REAR_LEFT), readEncoder(REAR_RIGHT));
  // }
}

// eliminates kick due to wheel movement since last PID loop.
// The kick comes because the encoder value is at the point
// when motor PWM becomes 0, not when the motors actually stop
void forFixingStartupKick() {
  // leftPID.Encoder = readEncoder(LEFT); // do not uncomment, it has an acceleration bug when driving
  // rightPID.Encoder = readEncoder(RIGHT); // do not uncomment, it has an acceleration bug when driving
  // rearLeftPID.Encoder = readEncoder(REAR_LEFT); // do not uncomment, it has an acceleration bug when driving
  // rearRightPID.Encoder = readEncoder(REAR_RIGHT); // do not uncomment, it has an acceleration bug when driving

  leftPID.PrevEnc = leftPID.Encoder;
  rightPID.PrevEnc = rightPID.Encoder;
  rearLeftPID.PrevEnc = rearLeftPID.Encoder;
  rearRightPID.PrevEnc = rearRightPID.Encoder;
}

#ifdef tuning_pid
// first send an m value then this function will do the rest
void forTuningPID() {

  // u 70:150:0:50 // tuned on 07/09/2925
  // u 320:10:0:60
  // u 100:70:0:20
  // u 100:75:1:20
  // u 105:75:1:22
  // u 105:80:1:18
  // u 105:80:1:20
  // u 110:85:1:10

  // u 500:260:0:50 // tuned on 08/09/2025
  // u 200:120:0:50
  // u 200:120:1:50

  // u 65:75:15:5 // for small signals
  // u 95:85:15:5
  // u 60:50:5:5
  // u 45:300:36:30 // diverges on small values

  // m 20 u 105:75:0:22 causes madness reducing Ko to 18 fixed it

  static unsigned long lastPID = 0;
  lastMotorCommand = millis();
  if (millis() - lastPID >= 2000) {
    lastPID = millis();
    moving = 1;

    leftPID.Encoder = readEncoder(LEFT);
    rightPID.Encoder = readEncoder(RIGHT);
    rearLeftPID.Encoder = readEncoder(REAR_LEFT);
    rearRightPID.Encoder = readEncoder(REAR_RIGHT);

    leftPID.PrevEnc = leftPID.Encoder;
    rightPID.PrevEnc = rightPID.Encoder;
    rearLeftPID.PrevEnc = rearLeftPID.Encoder;
    rearRightPID.PrevEnc = rearRightPID.Encoder;

    leftPID.TargetTicksPerFrame = 0;
    rightPID.TargetTicksPerFrame = 0;
    rearLeftPID.TargetTicksPerFrame = 0;
    rearRightPID.TargetTicksPerFrame = 0;

    tuning_pid.TargetTicksPerFrame = -tuning_pid.TargetTicksPerFrame;
  }
}

#endif

/*
// not related to project but might be useful in future
void runEvery(int interval, unsigned long* lastRunTime, void (*callback)()) {
  if (millis() < *lastRunTime + interval) {
    return;
  }
  *lastRunTime = millis();
  callback();
}
*/