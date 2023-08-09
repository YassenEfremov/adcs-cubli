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
#define ACCEL_CONFIG 0x1C   // Accel configuration address
#define GYRO_CONFIG 0x1B    // Gyro configuration address
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

// Sensor output scaling
#define accSens 0   // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1  // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

/*=================================================================*/

// offsets of gyros while balancing
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

struct Vec3_16 {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct Vec3_32 {
    int32_t x;
    int32_t y;
    int32_t z;
};

struct Vec3_f {
    float x;
    float y;
    float z;
};

enum BalancingPoint {
    NONE,
    CORNER,
    EDGE_BACK_R,
    EDGE_BACK_L,
    EDGE_FRONT
};

/*=================================================================*/

bool vertical = false;
bool calibrating = false;
bool calibrated = false;

BalancingPoint balancing_point = NONE;

float pGain = 150;
float iGain = 14.00;
float sGain = 0.035;

int loop_time = 10; // sampling rate of sensors in miliseconds - set by register 25

OffsetsObj offsets;

Vec3_16 accel_raw;
Vec3_16 gyro_raw;
Vec3_16 gyro_filt;

// user calibrated gyroscope offsets
Vec3_16 calibrated_offset{0.0, 0.0, 0.0};

// int32_t GyX_offset_sum = 0;  // not used?

Vec3_f robot_angle;
Vec3_f rotation;
Vec3_32 motor_speed_pwm;

int battery_divider = 57;

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
        tune(); // derinimui
        angle_calc();

        switch (balancing_point) {
            case CORNER:
                rotation.x -= offsets.X1;
                rotation.y -= offsets.Y1;
                if (abs(rotation.x) > 8 || abs(rotation.y) > 8)
                    vertical = false;
                break;

            case EDGE_BACK_R:
                rotation.x -= offsets.X2;
                rotation.y -= offsets.Y2;
                if (abs(rotation.y) > 6)
                    vertical = false;
                break;

            case EDGE_BACK_L:
                rotation.x -= offsets.X3;
                rotation.y -= offsets.Y3;
                if (abs(rotation.y) > 6)
                    vertical = false;
                break;

            case EDGE_FRONT:
                rotation.x -= offsets.X4;
                rotation.y -= offsets.Y4;
                if (abs(rotation.x) > 6)
                    vertical = false;
                break;
        }

        if (vertical && calibrated && !calibrating) {
            digitalWrite(BRAKE, HIGH);
            //int16_t gyroX;    // not used?
            int16_t gyroZ = gyro_raw.z / 131.0; // Convert to deg/s?
            int16_t gyroY = gyro_raw.y / 131.0; // Convert to deg/s?

            // exponential smoothing of the gyro readings
            float alpha = 0.6;
            gyro_filt.y = alpha * gyroY + (1 - alpha) * gyro_filt.y;
            gyro_filt.z = alpha * gyroZ + (1 - alpha) * gyro_filt.z;

            // Linear quadratic reguator - to read up on :)
            int pwm_X = constrain(pGain * rotation.x + iGain * gyro_filt.z + sGain * motor_speed_pwm.x, -255, 255);
            int pwm_Y = constrain(pGain * rotation.y + iGain * gyro_filt.y + sGain * motor_speed_pwm.y, -255, 255);
            motor_speed_pwm.x += pwm_X;
            motor_speed_pwm.y += pwm_Y;

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
            balancing_point = NONE;
            XY_to_threeWay(0, 0);
            digitalWrite(BRAKE, LOW);
            motor_speed_pwm.x = 0;
            motor_speed_pwm.y = 0;
        }
        previousT_1 = currentT;
    }

    if (currentT - previousT_2 >= 2000) {
        battVoltage((double)analogRead(VBAT) / battery_divider);
        if (!calibrated && !calibrating) {
            Serial.println("first you need to calibrate the balancing points...");
        }
        previousT_2 = currentT;
    }
}
