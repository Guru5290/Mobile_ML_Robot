// 988 cpr

// python3 -m serial.tools.miniterm /dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Mega_2560_12148509806232150650-if00 57600 --echo
// python3 -m serial.tools.miniterm /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 57600 --echo
// "/home/l/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/bin/avrdude" "-C/home/l/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/etc/avrdude.conf" -v -V -patmega2560 -cwiring "-P/dev/ttyACM0" -b115200 -D "-Uflash:w:/tmp/arduino/sketches/71D686EBAB249562A9FB6FE352DD37F9/ROSArduinoBridge.ino.hex:i"

#define USE_BASE  // Enable the base controller code
// #undef USE_BASE     // Disable the base controller code

#ifdef USE_BASE

#define OURENCODER

#define L298_MOTOR_DRIVER

#define BNO055

#endif

// #define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS  // Disable use of PWM servos

#define BAUDRATE 57600

#define MAX_PWM 255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "commands.h"
#include "sensors.h"

#ifdef USE_SERVOS
#include <Servo.h>
#include "servos.h"
#endif

#ifdef USE_BASE

#include "motor_driver.h"
#include "encoder_driver.h"
#include "diff_controller.h"
// #include "BNO055.h"
#include "GYVL530L0.h"
#include "GY87.h"
#include <SoftwareSerial.h>
// #include "mySerial.h"

#define PID_RATE 30  // Hz
// #define PID_RATE 50  // Hz
#define SENSOR_SAMPLERATE_DELAY_MS 100

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

#define RX 12
#define TX 13

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int inde_x = 0;

// Variable to hold an input character
char chr;
char mySerialchr;

// Variable to hold the current single-character command
char cmd;
char mySerialCmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

// Start-Stop IMU
int pub_IMU_TOF = 0;
SoftwareSerial mySerial(RX, TX);

/* Clear the current command parameters */
void resetCommand() {
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
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
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        resetPID();
        moving = 0;
      } else
        moving = 1;
      leftPID.TargetTicksPerFrame = arg1;
      rightPID.TargetTicksPerFrame = arg2;
      Serial.println("OK");
      break;
    case MOTOR_RAW_PWM:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      resetPID();
      moving = 0;  // Sneaky way to temporarily disable the PID
      setMotorSpeeds(arg1, arg2);
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
      Serial.print(Kp);
      Serial.print(" ");
      Serial.print(Kd);
      Serial.print(" ");
      Serial.print(Ki);
      Serial.print(" ");
      Serial.println(Ko);
      break;

    case GO_FORWARD:
      lastMotorCommand = millis();
      setMotorSpeedsFeedback(60, 60);
      Serial.println("OK");
      break;

    case GO_BACKWARD:
      lastMotorCommand = millis();
      setMotorSpeedsFeedback(-60, -60);
      Serial.println("OK");
      break;

    case TURN_LEFT:
      lastMotorCommand = millis();
      setMotorSpeedsFeedback(-30, 30);
      Serial.println("OK");
      break;

    case TURN_RIGHT:
      lastMotorCommand = millis();
      setMotorSpeedsFeedback(30, -30);
      Serial.println("OK");
      break;
    case IMU_DATA:
      pub_IMU_TOF = 1;
      Serial.println("OK");
      break;
    case STOP_IMU:
      pub_IMU_TOF = 0;
      Serial.println("OK");
      break;

#endif
    default:
      Serial.println("Invalid Command Serial");
      // mySerial.println("Invalid Command Serial " + String((int)cmd));
      break;
  }
}

// Supports only single letter commands
void runCommandMySerial(char command) {
  switch (command) {
    case IMU_DATA:
      pub_IMU_TOF = 1;
      mySerial.println("OK");
      break;
    case STOP_IMU:
      pub_IMU_TOF = 0;
      mySerial.println("OK");
      break;

    default:
      mySerial.println("Invalid Command mySerial");
      break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
  mySerial.begin(BAUDRATE);
  // initIMU();
  // initBMP();
  initMPU6050();
  initTOF();

// Initialize the motor controller if used */
#ifdef USE_BASE

#ifdef OURENCODER

  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), countRShort, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), countLShort, CHANGE);

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
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {

  if (pub_IMU_TOF == 1) {
    // IMU();
    // BMP();
    mpu6050();
    TOF();
    printOut();
  }
  

  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    if (chr == 10 || chr == 13) {  // terminate with \n or \r
      if (arg == 1)
        argv1[inde_x] = '\0';
      else if (arg == 2)
        argv2[inde_x] = '\0';
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
      }
    }
  }

  // only single character commands are supported here
  while (mySerial.available() > 0) {

    // Read the next character
    mySerialchr = mySerial.read();

    // if (mySerialchr == 10) {  // terminate with \n
    if (mySerialchr == 10 || mySerialchr == 13) {  // terminate with \n or \r
      runCommandMySerial(mySerialCmd);
      // resetCommand();
      mySerialCmd = 0;

      while (mySerial.available()) {  // to clear mySerial buffer
        mySerial.read();
      }
    } else {
      mySerialCmd = mySerialchr;
    }
  }

// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}

String formattedReadings = "";
unsigned long lastPrinted = 0;
void printOut() {
  if (millis() < lastPrinted + SENSOR_SAMPLERATE_DELAY_MS) {
    return;
  }
  
  lastPrinted= millis();
  formattedReadings = String(tofReading) + " " + String(mpuReading[0])+ " " + String(mpuReading[1])+ " " + String(mpuReading[2]);
  mySerial.println(formattedReadings);
}

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