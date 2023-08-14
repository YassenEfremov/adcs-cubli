// Motor connection pins


#define PWM_1         11
#define DIR_1         10
#define SPD_1         9

#define PWM_2         13
#define DIR_2         7
#define SPD_2         0

#define PWM_3         3     //speed of motor
#define DIR_3         4     //direction of motor
#define SPD_3         0

// other pins
#define BRAKE         8
#define BUZZER        12
#define VBAT          A7    //used to check battery voltage (11.1V)

#define MPU6050 0x68          // MPU6050 address for I2C
#define ACCEL_CONFIG 0x1C     // Accelerometer configuration register address
#define GYRO_CONFIG 0x1B      // Gyro configuration address
#define PWR_MGMT_1 0x6B       // Power Management Register 1
#define PWR_MGMT_2 0x6C

//Sensor output scaling
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s


struct OffsetsObj {
  int ID1;
  float X1;
  float Y1;
  int ID2;
  float X2;
  float Y2;
  int ID3;
  float X3;
  float Y3;
  int ID4;
  float X4;
  float Y4;
};

OffsetsObj offsets;

float alpha = 0.6;              // used in filetering gyroscope readings

int16_t  AcX, AcY, AcZ;         //storage of raw accelerometer readings
int16_t  GyX, GyY, GyZ;          //storage of raw gyroscope readings
int16_t  gyroX, gyroY, gyroZ;    // 
int16_t  gyroXfilt,gyroYfilt, gyroZfilt;  // storage of filtered gyroscope reading 

int16_t  GyX_offset = 0;        // offset of each gyroscope reading
int16_t  GyZ_offset = 0;        
int16_t  GyY_offset = 0;

int16_t AcX_offset = 0;
int16_t AcY_offset = 0;
int16_t AcZ_offset = 0;

int32_t AcX_offset_sum = 0;
int32_t AcY_offset_sum = 0;
int32_t AcZ_offset_sum = 0;

int32_t  GyX_offset_sum = 0;    //used to calculate the average offsets
int32_t  GyZ_offset_sum = 0;
int32_t  GyY_offset_sum = 0;

float Gyro_amount = 0.996;  

bool vertical = false;
bool calibrating = false;
bool calibrated = false;

int balancing_point = 3;

float pGain =30 ;
float iGain = 14;
float sGain = 0.1;


int loop_time = 10;

int MMS = 255/sGain;


float K1= 200;
float K2= 30;
float K3= 0.035;

float K4=0.1;

float mot_V_X, mot_V_Y;
int pwm_X,pwm_Y;
float robot_angleX, robot_angleY, angleX, angleY;
float Acc_angleX, Acc_angleY;      
int32_t motor_speed_pwmX; 
int64_t motor_speed_pwmY;   

int bat_divider = 57;

long currentT, previousT_1, previousT_2 = 0;

float theta1_X;
float theta1_Y;
float theta1_Z;

float theta1dot_X;
float theta1dot_Y;
float theta1dot_Z;

float theta2dot_1;
float theta2dot_2;
float thera2dot_3;

  int pin;
  int i = 0;
  int j = 0;