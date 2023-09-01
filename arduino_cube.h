#include <MadgwickAHRS.h>


//pins for motors
#define PWM_1		3	// new: 3	og: 10
#define DIR_1		2	// new: 2	og: 8

#define PWM_2		5	// new: 5	og: 9
#define DIR_2		4	// new: 4	og: 7

#define PWM_3		6	// new: 6	og: 5
#define DIR_3		7	// new: 7	og: 4

						// 8 - AltSoftSerial Rx
						// 9 - AltSoftSerial Tx
						// 10 - AltSoftSerial Unusable PWM

// other pins
#define BRAKE		11
#define BUZZER		12

//IMU register values
#define MPU6050 0x68          // MPU6050 address for I2C
#define ACCEL_CONFIG 0x1C     // Accelerometer configuration register address
#define GYRO_CONFIG 0x1B      // Gyro configuration address
#define PWR_MGMT_1 0x6B       // Power Management Register 1
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 0            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

//control parameters
#define loop_time  10  // the sample rate in ms

//madgwick filter properties
#define mgwk_freq 1000         //sampling frequency of the filter


//Physical system parameters
#define max_spd 16.65




// Control system variables
//##############################################################################
float K1=  15;  // gains theta1 //30/
float K2=  1;        // theta1dot //6
float K3=  0.000001;      // theta2   //0.15//0.05

//madgwick filter and sensor variables
//############################################################################
Madgwick filter;
long millis_per_filter_reading;
float AcX, AcY, AcZ;         //storage of raw accelerometer readings
float AcX_offset, AcY_offset, AcZ_offset;
float AcX_offset_sum, AcY_offset_sum, AcZ_offset_sum;


float GyX, GyY, GyZ;          //storage of raw gyroscope readings 
float GyX_offset, GyY_offset, GyZ_offset;
float GyX_offset_sum, GyY_offset_sum, GyZ_offset_sum;

// Sensor derived data variables
//############################################################################
float theta1_X = 0;       // system angular position (rad)
float theta1dot_X= 0;    // system angular velocity

float theta1_Y = 0;      
float theta1dot_Y= 0;    

float theta2dot_1=0;
float theta2dot_2=0;
float theta2dot_3=0;

float req_torque1;      // torque required by the control system
float req_torque2; 
float req_torque3; 

float req_acc1;          //that torque represented in acceleration
float req_acc2; 
float req_acc3; 

//general timer readings
//############################################################################
long currentT, previousT_1; //timer interrupts

//tuning paprameters
char TxRx = ' ';