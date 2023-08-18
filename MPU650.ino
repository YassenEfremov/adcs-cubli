// MPU6050 addresses


void MPU6050_init() //initialize the sensor
{
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);              // specify internal 8MHz occilator?
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);
}

void MPU6050_read_gyros()
{
    // read raw accel/gyro measurements from device - check the register map pg 31
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);   //dont know why 6 bytes are being requested
  GyX = ((Wire.read() << 8 | Wire.read())-GyX_offset); // 0x43 (GYRO_XOUT_H) bitwise-or 0x44 (GYRO_XOUT_L)
  GyY = (Wire.read() << 8 | Wire.read())-GyY_offset; // 0x45 (GYRO_YOUT_H) bitwise-or 0x46 (GYRO_YOUT_L)
  GyZ = (Wire.read() << 8 | Wire.read())-GyZ_offset; // 0x47 (GYRO_ZOUT_H) bitwise-or 0x48 (GYRO_ZOUT_L)
}

void MPU6050_read_accelerometers()
{
    Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  AcX = ((Wire.read() << 8 | Wire.read())); // 0x3B (ACCEL_XOUT_H) bitwise-or 0x3C (ACCEL_XOUT_L)
  AcY = ((Wire.read() << 8 | Wire.read())); // 0x3D (ACCEL_YOUT_H) bitwise-or 0x3E (ACCEL_YOUT_L)
  AcZ = ((Wire.read() << 8 | Wire.read())); // 0x3F (ACCEL_ZOUT_H) bitwise-or 0x40 (ACCEL_ZOUT_L)
}

void MPU6050_filter_sensors()     //gyrofilt applies the LSB accuracy and makes the readings more stable
{    
      GyXF = alpha * (GyX/65.5) + (1 - alpha) * GyXF; //note gyrofilt is still in deg/s

      AcXF = (alpha * (AcX/16384)+ (1 - alpha) * AcXF);
      AcYF = (alpha * (AcY/16384)+ (1 - alpha) * AcYF);
      AcZF = (alpha * (AcZ/16384)+ (1 - alpha) * AcZF);    
}