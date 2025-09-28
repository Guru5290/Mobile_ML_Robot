#ifdef L298_MOTOR_DRIVER

#define FRONT_LEFT_MOTOR_BACKWARD PB5   // T3C2 4
#define FRONT_LEFT_MOTOR_FORWARD PB4    // T3C1
#define FRONT_RIGHT_MOTOR_BACKWARD PA1  // T5C1 1
#define FRONT_RIGHT_MOTOR_FORWARD PA0   // T5C2
#define REAR_LEFT_MOTOR_BACKWARD PA2    // T5C3 3
#define REAR_LEFT_MOTOR_FORWARD PA3     // T5C4
#define REAR_RIGHT_MOTOR_BACKWARD PB8   // T4C3 2
#define REAR_RIGHT_MOTOR_FORWARD PB9    // T4C4

#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed, int rearLeftSpeed, int rearRightSpeed);
