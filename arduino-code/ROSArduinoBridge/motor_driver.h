#ifdef L298_MOTOR_DRIVER

#define RIGHT_MOTOR_FORWARD 5 //8
#define RIGHT_MOTOR_BACKWARD 9
#define LEFT_MOTOR_FORWARD 11
#define LEFT_MOTOR_BACKWARD 10
//#define LEFT_MOTOR_ENABLE 5
//#define RIGHT_MOTOR_ENABLE 6

#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
