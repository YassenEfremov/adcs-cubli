//pins for motor 1
#define PWM_1         11
#define DIR_1         10
#define SPD_1         9

// other pins
#define BRAKE         8
#define BUZZER        12

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
float K1=   30.8;  // gains theta1 //30/
float K2=  1;        // theta1dot //6
float K3=  0.02;      // theta2   //0.15//0.05

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
float theta2dot_1 = 0;    // system flywheel angular velocity
float req_torque;      // torque required by the control system
float req_acc;          //that torque represented in acceleration
//general timer readings
//############################################################################
long currentT, previousT_1; //timer interrupts

//tuning paprameters
char TxRx;