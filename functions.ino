/**
 * writes data to a device address
 */
void writeTo(byte device, byte address, byte value)   // simplifies writing to registers
{
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.write(value);
    Wire.endTransmission(true);
}


/**
 * makes a beep sound
 */
void beep()
{
    digitalWrite(BUZZER, HIGH);
    delay(70);
    digitalWrite(BUZZER, LOW);
    delay(80);
}


/**
 * saves offset values to memory and determines calibrating
 */
void save()
{
    EEPROM.put(0, offsets);
    EEPROM.get(0, offsets);
    if (offsets.ID1 == 99 && offsets.ID2 == 99 && offsets.ID3 == 99 && offsets.ID4 == 99)
        calibrated = true;
    calibrating = false;
    Serial.println("calibrating off");
    beep();
}


/**
 * sets internal oscillator, scaling and ....?
 */
void angle_setup()
{
    Wire.begin();
    delay(100);
    writeTo(MPU6050, PWR_MGMT_1, 0);              // specify internal 8MHz occilator?
    writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specify output scaling of accelerometer
    writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specify output scaling of gyroscope
    delay(100);

    int32_t gyro_Z_offset_sum = 0;
    for (int i = 0; i < 1024; i++) {
        angle_calc();
        // Serial.println(gyro_raw.z);
        gyro_Z_offset_sum += gyro_raw.z;
        delay(3);
    }
    calibrated_offset.z = gyro_Z_offset_sum >> 10;  // average z offset
    Serial.print("GyZ offset value = ");
    Serial.println(calibrated_offset.z);
    beep();

    int32_t gyro_Y_offset_sum = 0;
    for (int i = 0; i < 1024; i++) {
        angle_calc();
        // Serial.println(gyro_raw.y);
        gyro_Y_offset_sum += gyro_raw.y;
        delay(3);
    }
    calibrated_offset.y = gyro_Y_offset_sum >> 10;  // average y offset
    Serial.print("GyY offset value = ");
    Serial.println(calibrated_offset.y);
    beep();
    beep();
}


/**
 * determines our x and y rotation
 */
void angle_calc()
{
    // read raw gyro measurements from device (deg/s) - check the register map pg 31
    Wire.beginTransmission(MPU6050);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 6, true);
    //GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) bitwise-or 0x44 (GYRO_XOUT_L)
    gyro_raw.y = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gyro_raw.z = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    // read raw accel measurements from device - (g) check the register map pg 29
    Wire.beginTransmission(MPU6050);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 6, true);
    accel_raw.x = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    accel_raw.y = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    accel_raw.z = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

    // adjust the readings of the gyros by subracting the manually calibrated offsets
    gyro_raw.z -= calibrated_offset.z;
    gyro_raw.y -= calibrated_offset.y;

    // determine discrete difference in change
    robot_angle.x += gyro_raw.z * loop_time / 1000 / 65.536; //65.536 is lsb sensitivity //May have to be GyX ?????????????????? //roll :)
    robot_angle.y += gyro_raw.y * loop_time / 1000 / 65.536;// //pitch


    // determine the angle of the robot about the x & y axis (degrees) (pitch and roll)
    float gyro_amount = 0.996; // determines the ratio of how much the code that adds the gyro and acc readings together to determine discrete increase in angle change (functions line 97) 
    float acc_angleX = atan2(accel_raw.y, -accel_raw.x) * 57.2958;    
    robot_angle.x = robot_angle.x * gyro_amount + acc_angleX * (1.0 - gyro_amount); // not sure why they are adding the accelerometer angle  
    float acc_angleY = -atan2(accel_raw.z, -accel_raw.x) * 57.2958;
    robot_angle.y = robot_angle.y * gyro_amount + acc_angleY * (1.0 - gyro_amount);

    rotation.x = robot_angle.x;
    rotation.y = robot_angle.y;

    is_it_balancing();
}

/**
 * determines the balancing point we are closest to
 */
void is_it_balancing()
{
    if (abs(rotation.x - offsets.X1) < 0.4 && abs(rotation.y - offsets.Y1) < 0.4)
    {
        balancing_point = CORNER;
        if (!vertical)
            beep();
        vertical = true;
    }
    else if (abs(rotation.x - offsets.X2) < 3 && abs(rotation.y - offsets.Y2) < 0.6)
    {
        balancing_point = EDGE_BACK_R;
        if (!vertical)
            beep();
        vertical = true;
    }
    else if (abs(rotation.x - offsets.X3) < 6 && abs(rotation.y - offsets.Y3) < 0.6) /// maybe change from 6?
    {
        balancing_point = EDGE_BACK_L;
        if (!vertical)
            beep();
        vertical = true;
    }
    else if (abs(rotation.x - offsets.X4) < 0.6 && abs(rotation.y - offsets.Y4) < 3)
    {
        balancing_point = EDGE_FRONT;
        if (!vertical)
            beep();
        vertical = true;
    }
}


/**
 * balances on CORNER
 */
void XY_to_threeWay(float pwm_X, float pwm_Y)
{
    int16_t m1 = round(0.5 * pwm_X - 0.75 * pwm_Y);
    int16_t m2 = round(0.5 * pwm_X + 0.75 * pwm_Y);
    int16_t m3 = pwm_X;

    m1 = constrain(m1, -255, 255);
    m2 = constrain(m2, -255, 255);
    m3 = constrain(m3, -255, 255);

    Motor1_control(-m1);
    Motor2_control(-m2);
    Motor3_control(m3);
}


/**
 * battery status
 */
void battVoltage(double voltage)
{
    // Serial.print("batt: "); Serial.println(voltage); //debug
    if (voltage > 8 && voltage <= 9.5)
    {
        digitalWrite(BUZZER, HIGH);
    }
    else
    {
        digitalWrite(BUZZER, LOW);
    }
}


/**
 * controls motor 1 speed and direction
 */
void Motor1_control(int sp)
{
    if (sp > 0)
        digitalWrite(DIR_1, LOW);
    else
        digitalWrite(DIR_1, HIGH);

    analogWrite(PWM_1, 255 - abs(sp));
}


/**
 * controls motor 2 speed and direction
 */
void Motor2_control(int sp)
{
    if (sp > 0)
        digitalWrite(DIR_2, LOW);
    else
        digitalWrite(DIR_2, HIGH);

    analogWrite(PWM_2, 255 - abs(sp));
}


/**
 * controls motor 3 speed and direction
 */
void Motor3_control(int sp)
{
    if (sp > 0)
        digitalWrite(DIR_3, LOW);
    else
        digitalWrite(DIR_3, HIGH);

    analogWrite(PWM_3, 255 - abs(sp));
}


/**
 * allows tuning the PID coefficients over cable or bluetooth
 */
int tune()
{
    if (!Serial.available())
        return 0;
    delay(2);
    char param = Serial.read(); // get parameter byte
    if (!Serial.available())
        return 0;
    char cmd = Serial.read(); // get command byte
    Serial.flush();

    switch (param) {
    case 'p':
        if (cmd == '+')
            pGain += 1;
        if (cmd == '-')
            pGain -= 1;
        printValues();
        break;
    case 'i':
        if (cmd == '+')
            iGain += 0.05;
        if (cmd == '-')
            iGain -= 0.05;
        printValues();
        break;
    case 's':
        if (cmd == '+')
            sGain += 0.005;
        if (cmd == '-')
            sGain -= 0.005;
        printValues();
        break;
    case 'b':
        if (cmd == '+')
            battery_divider += 1;
        if (cmd == '-')
            battery_divider -= 1;
        printValues();
        break;
    case 'c':
        if (cmd == '+' && !calibrating)
        {
            calibrating = true;
            Serial.println("calibrating on");
        }
        if (cmd == '-' && calibrating)
        {
            Serial.print("X: ");
            Serial.print(robot_angle.x);
            Serial.print(" Y: ");
            Serial.println(robot_angle.y);
            if (abs(robot_angle.x) < 10 && abs(robot_angle.y) < 10)
            {
                offsets.ID1 = 99;
                offsets.X1 = robot_angle.x;
                offsets.Y1 = robot_angle.y;
                Serial.println("Vertex OK.");
                save();
            }
            else if (robot_angle.x > -45 && robot_angle.x < -25 && robot_angle.y > -30 && robot_angle.y < -10)
            {
                offsets.ID2 = 99;
                offsets.X2 = robot_angle.x;
                offsets.Y2 = robot_angle.y;
                Serial.println("First edge OK.");
                save();
            }
            else if (robot_angle.x > 20 && robot_angle.x < 40 && robot_angle.y > -30 && robot_angle.y < -10)
            {
                offsets.ID3 = 99;
                offsets.X3 = robot_angle.x;
                offsets.Y3 = robot_angle.y;
                Serial.println("Second edge OK.");
                save();
            }
            else if (abs(robot_angle.x) < 15 && robot_angle.y > 30 && robot_angle.y < 50)
            {
                offsets.ID4 = 99;
                offsets.X4 = robot_angle.x;
                offsets.Y4 = robot_angle.y;
                Serial.println("Third edge OK.");
                save();
            }
            else
            {
                Serial.println("The angles are wrong!!!");
                beep();
                beep();
            }
        }
        break;
    }
}


void printValues()
{
    Serial.print("P: ");
    Serial.print(pGain);
    Serial.print(" I: ");
    Serial.print(iGain);
    Serial.print(" S: ");
    Serial.println(sGain, 4);
    Serial.print("battery_divider: ");
    Serial.println(battery_divider);
}
