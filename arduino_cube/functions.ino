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

    int32_t GyZ_offset_sum = 0;
    for (int i = 0; i < 1024; i++) {
        angle_calc();
        // Serial.println(GyZ);
        GyZ_offset_sum += GyZ;
        delay(3);
    }
    GyZ_offset = GyZ_offset_sum >> 10;
    Serial.print("GyZ offset value = ");
    Serial.println(GyZ_offset);
    beep();

    int32_t GyY_offset_sum = 0;
    for (int i = 0; i < 1024; i++) {
        angle_calc();
        // Serial.println(GyY);
        GyY_offset_sum += GyY;
        delay(3);
    }
    GyY_offset = GyY_offset_sum >> 10;
    Serial.print("GyY offset value = ");
    Serial.println(GyY_offset);
    beep();
    beep();
}


/**
 * determines the balancing point and some other things
 */
void angle_calc()
{
    // read raw accel/gyro measurements from device (deg/s) - check the register map pg 31
    Wire.beginTransmission(MPU6050);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 6, true);
    //GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) bitwise-or 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    // read raw accel/gyro measurements from device - (g) check the register map pg 29
    Wire.beginTransmission(MPU6050);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 6, true);
    AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

    // adjust the readings of the gyros by subracting the manually calibrated offsets
    // GyX -= GyX_offset;// and this
    GyZ -= GyZ_offset;
    GyY -= GyY_offset;

    // determine discrete difference in change
    robot_angleX += GyZ * loop_time / 1000 / 65.536; //65.536 is lsb sensitivity //May have to be GyX ?????????????????? //roll :)
    robot_angleY += GyY * loop_time / 1000 / 65.536;// //pitch
    // robot_angleZ += GyX * loop_time / 1000/ 65.536;   i added this


    // deterine the angle of the robot about the x & y axis (degrees) (pitch and roll)
    float Gyro_amount = 0.996; // determines the ratio of how much the code that adds the gyro and acc readings together to determine discrete increase in angle change (functions line 97) 
    Acc_angleX = atan2(AcY, -AcX) * 57.2958;    
    robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount); // not sure why they are adding the accelerometer angle  
    Acc_angleY = -atan2(AcZ, -AcX) * 57.2958;
    robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

    angleX = robot_angleX;  //
    angleY = robot_angleY;  //

    is_it_balancing();
}

/**
 * determines the balancing point we are closest to
 */
void is_it_balancing()
{
    if (abs(angleX - offsets.X1) < 0.4 && abs(angleY - offsets.Y1) < 0.4)
    {
        balancing_point = CORNER;
        if (!vertical)
            beep();
        vertical = true;
    }
    else if (abs(angleX - offsets.X2) < 3 && abs(angleY - offsets.Y2) < 0.6)
    {
        balancing_point = EDGE_BACK_R;
        if (!vertical)
            beep();
        vertical = true;
    }
    else if (abs(angleX - offsets.X3) < 6 && abs(angleY - offsets.Y3) < 0.6) /// maybe change from 6?
    {
        balancing_point = EDGE_BACK_L;
        if (!vertical)
            beep();
        vertical = true;
    }
    else if (abs(angleX - offsets.X4) < 0.6 && abs(angleY - offsets.Y4) < 3)
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
 * ?
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

void Motor1_control(int sp) // control motor speed and direction
{
    if (sp > 0)
        digitalWrite(DIR_1, LOW);
    else
        digitalWrite(DIR_1, HIGH);

    analogWrite(PWM_1, 255 - abs(sp));
}

void Motor2_control(int sp)
{
    if (sp > 0)
        digitalWrite(DIR_2, LOW);
    else
        digitalWrite(DIR_2, HIGH);

    analogWrite(PWM_2, 255 - abs(sp));
}

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
int tuning()
{
    if (!Serial.available())
        return 0;
    delay(2);
    char param = Serial.read(); // get parameter byte
    if (!Serial.available())
        return 0;
    char cmd = Serial.read(); // get command byte
    Serial.flush();
    switch (param)
    {
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
            bat_divider += 1;
        if (cmd == '-')
            bat_divider -= 1;
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
            Serial.print(robot_angleX);
            Serial.print(" Y: ");
            Serial.println(robot_angleY);
            if (abs(robot_angleX) < 10 && abs(robot_angleY) < 10)
            {
                offsets.ID1 = 99;
                offsets.X1 = robot_angleX;
                offsets.Y1 = robot_angleY;
                Serial.println("Vertex OK.");
                save();
            }
            else if (robot_angleX > -45 && robot_angleX < -25 && robot_angleY > -30 && robot_angleY < -10)
            {
                offsets.ID2 = 99;
                offsets.X2 = robot_angleX;
                offsets.Y2 = robot_angleY;
                Serial.println("First edge OK.");
                save();
            }
            else if (robot_angleX > 20 && robot_angleX < 40 && robot_angleY > -30 && robot_angleY < -10)
            {
                offsets.ID3 = 99;
                offsets.X3 = robot_angleX;
                offsets.Y3 = robot_angleY;
                Serial.println("Second edge OK.");
                save();
            }
            else if (abs(robot_angleX) < 15 && robot_angleY > 30 && robot_angleY < 50)
            {
                offsets.ID4 = 99;
                offsets.X4 = robot_angleX;
                offsets.Y4 = robot_angleY;
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
    Serial.print("Bat_divider: ");
    Serial.println(bat_divider);
}
