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

//system parameters
#define g  9.81                  // gravity biatch
#define L  0.10606               //reaction wheel to fixing point (m)
#define l  0.07                  //centre of mass to fixing point (m)
#define M  0.1                  //reaction wheel mass (kg)
#define m  0.3                   //pedulum mass (kg)

#define Bmot  0.2                    //friction constant (N.m.s)
#define Kmot 1e-3                    //Back EMF and Motor Torque constant (Nm/A)
#define R  1                         //electric resistance
#define L_mot 0.5                    //electric inductance
#define max_torque  0.2353           // max torque of the motor
#define max_spd 16.65

//control parameters
#define loop_time  2  // the sample rate in ms

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
// Physical system variables

float I_wc = 0;      //reaction wheel inertia (kgm2)
float I_pb = 0;              //pendulum inertia (kgm2)
float max_acc=0;    //assumed based on power (hence the 2)



// Control system variables
//##############################################################################
float K1= 7 ;     // gains theta1
float K2= 14;        // theta1dot
float K3= 0;      // theta2
float K4 = 2;
float Gyro_amount = 0.85;  
// Sensor data variables
//##############################################################################
float alpha = 0.6;              // used in filetering gyroscope readings
float AcX, AcY, AcZ;         //storage of raw accelerometer readings
float  AcXF, AcYF, AcZF;         //storage of filtered accelerometer readings
float  GyX, GyY, GyZ;          //storage of raw gyroscope readings
float  GyXF,GyYF,GyZF;        //storage of filtered gyroscope readings 

// offset of each gyroscope and accelerometer reading
int16_t  GyX_offset = 0;        
int16_t  GyZ_offset = 0;        
int16_t  GyY_offset = 0;
int16_t AcX_offset = 0;
int16_t AcY_offset = 0;
int16_t AcZ_offset = 0;
//used to calculate the average offsets  
int32_t AcX_offset_sum = 0;   
int32_t AcY_offset_sum = 0;
int32_t AcZ_offset_sum = 0;
int32_t  GyX_offset_sum = 0;    
int32_t  GyZ_offset_sum = 0;
int32_t  GyY_offset_sum = 0;

float angleX_offset = 0;
float angleX_offset_sum = 0;

// Sensor derived data variables
//############################################################################
float Acc_angleX;     // accelerometer angle reading
float Gyr_angleX;     // gyroscope angle reading
float theta1_X = 0;       // system angular position (rad)
float theta1dot_X= 0;    // system angular velocity
float theta2dot_1 = 0;    // system flywheel angular velocity
int32_t motor_speed_pwmX; // measuremnt of the PWM sent to the motor 

//Actuator Variables
//#############################################################################
float req_torque;      // torque required by the control system
float req_acc;

//System Variables
//#############################################################################
long currentT, previousT_1, previousT_2 = 0; //timer interrupts
