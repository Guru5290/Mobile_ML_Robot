// 988 cpr

#ifdef OURENCODER

// at least one interrupt-capable pin per motor
#define FRONT_LEFT_ENC_PIN_A PB13  //
#define FRONT_LEFT_ENC_PIN_B PB12  // 4
#define FRONT_RIGHT_ENC_PIN_A PB3  //
#define FRONT_RIGHT_ENC_PIN_B PB0  // 1
#define REAR_LEFT_ENC_PIN_A PA10   //
#define REAR_LEFT_ENC_PIN_B PA4    // 3
#define REAR_RIGHT_ENC_PIN_A PA15  //
#define REAR_RIGHT_ENC_PIN_B PC14  // 2


#endif
void countFrontRShort();
void countFrontLShort();
void countRearRShort();
void countRearLShort();
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
