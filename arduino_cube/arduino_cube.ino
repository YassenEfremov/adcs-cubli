// #include "arduino_cube.h"
#include <Wire.h>
#include <EEPROM.h>


#define PWM_3 3
#define DIR_3 4

#define PWM_1 10
#define DIR_1 2

#define PWM_2 9
#define DIR_2 7

#define BRAKE 8
#define BUZZER 12
#define VBAT A7

#define MPU6050 0x68        // Device address
#define ACCEL_CONFIG 0x1C   // Accelerometer configuration address
#define GYRO_CONFIG 0x1B    // Gyro configuration address
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

// Sensor output scaling
#define accSens 0   // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1  // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s


// these values are in a struct because they are read consecutively from memory
struct OffsetsObj // offsets of gyros while balancing
{
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

struct Vec3_16 {
    int16_t x;
    int16_t y;
    int16_t z;
};

enum BalancingPoint {
    CORNER = 1,
    EDGE_BACK_R = 2,
    EDGE_BACK_L = 3,
    EDGE_FRONT = 4
};


bool vertical = false;
bool calibrating = false;
bool calibrated = false;

BalancingPoint balancing_point = 0;

float pGain = 150;
float iGain = 14.00;
float sGain = 0.035;

int loop_time = 10; // sampling rate of sensors in miliseconds - set by register 25

OffsetsObj offsets;

int16_t AcX, AcY, AcZ;
int16_t GyY, GyZ;

// user calibrated gyroscope offsets
int16_t GyX_offset = 0;
int16_t GyZ_offset = 0;
int16_t GyY_offset = 0;

// int32_t GyX_offset_sum = 0;  // not used?

float robot_angleX, robot_angleY, angleX, angleY;
float Acc_angleX, Acc_angleY;
int32_t motor_speed_pwmX;
int32_t motor_speed_pwmY;

int bat_divider = 57;

/*=================================================================*/

void setup()
{
    Serial.begin(115200);

    // Set up timers 1 and 2 (to be determined why?)
    TCCR1A = 0b00000001;
    TCCR1B = 0b00001010;
    TCCR2B = 0b00000010;
    TCCR2A = 0b00000011;

    // sets motor diretion outputs and other digital pins
    pinMode(DIR_1, OUTPUT);
    pinMode(DIR_2, OUTPUT);
    pinMode(DIR_3, OUTPUT);
    pinMode(BRAKE, OUTPUT);
    pinMode(BUZZER, OUTPUT);

    // motors turn on and keeps high
    Motor1_control(0);
    Motor2_control(0);
    Motor3_control(0);

    EEPROM.get(0, offsets);   //read the offsets from the EEPROM
    if (offsets.ID1 == 99 && offsets.ID2 == 99 && offsets.ID3 == 99 && offsets.ID4 == 99) {
        calibrated = true;
    } else {
        calibrated = false;
    }
    delay(3000);
    beep();
    angle_setup();
}

/*=================================================================*/

void loop()
{
    long currentT, previousT_1, previousT_2 = 0;
    currentT = millis();

    if (currentT - previousT_1 >= loop_time) {
        tuning(); // derinimui
        angle_calc();

        switch (balancing_point) {
            case CORNER:
                angleX -= offsets.X1;
                angleY -= offsets.Y1;
                if (abs(angleX) > 8 || abs(angleY) > 8)
                    vertical = false;
                break;

            case EDGE_BACK_R:
                angleX -= offsets.X2;
                angleY -= offsets.Y2;
                if (abs(angleY) > 6)
                    vertical = false;
                break;

            case EDGE_BACK_L:
                angleX -= offsets.X3;
                angleY -= offsets.Y3;
                if (abs(angleY) > 6)
                    vertical = false;
                break;

            case EDGE_FRONT:
                angleX -= offsets.X4;
                angleY -= offsets.Y4;
                if (abs(angleX) > 6)
                    vertical = false;
                break;
        }

        if (vertical && calibrated && !calibrating) {
            digitalWrite(BRAKE, HIGH);
            //int16_t gyroX;    // not used?
            int16_t gyroZ = GyZ / 131.0; // Convert to deg/s?
            int16_t gyroY = GyY / 131.0; // Convert to deg/s?

            int16_t gyroYfilt, gyroZfilt;
            float alpha = 0.6;
            gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt; // no clue
            gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;

            int pwm_X = constrain(pGain * angleX + iGain * gyroZfilt + sGain * motor_speed_pwmX, -255, 255); // Linear quadratic reguator - to read up on :)
            int pwm_Y = constrain(pGain * angleY + iGain * gyroYfilt + sGain * motor_speed_pwmY, -255, 255); // LQR
            motor_speed_pwmX += pwm_X;
            motor_speed_pwmY += pwm_Y;

            // this switch tells which motors to move
            switch (balancing_point) {
                case CORNER:
                    XY_to_threeWay(-pwm_X, -pwm_Y);
                    break;

                case EDGE_BACK_R:
                    Motor1_control(-pwm_Y);
                    break;

                case EDGE_BACK_L:
                    Motor2_control(pwm_Y);
                    break;

                case EDGE_FRONT:
                    Motor3_control(-pwm_X);
                    break;
            }
        } else {
            balancing_point = 0;
            XY_to_threeWay(0, 0);
            digitalWrite(BRAKE, LOW);
            motor_speed_pwmX = 0;
            motor_speed_pwmY = 0;
        }
        previousT_1 = currentT;
    }

    if (currentT - previousT_2 >= 2000) {
        battVoltage((double)analogRead(VBAT) / bat_divider);
        if (!calibrated && !calibrating) {
            Serial.println("first you need to calibrate the balancing points...");
        }
        previousT_2 = currentT;
    }
}
