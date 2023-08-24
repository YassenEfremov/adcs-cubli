//initialize the sensor
void MPU6050_init() 
{
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);              // specify internal 8MHz occilator?
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);
}
void MPU6050_offset_setup() {  //, scaling and ......

  for (int i = 0; i <= 1000; i++) {
    MPU6050_read_gyros();
    GyX_offset_sum += GyX;
    GyY_offset_sum += GyY;
    GyZ_offset_sum += GyZ;
    MPU6050_read_accelerometers();
    AcX_offset_sum += AcX;
    AcY_offset_sum += AcY;
    AcZ_offset_sum += AcZ-1;
    delay(1);
  }
  GyX_offset = GyX_offset_sum/1000;
  GyY_offset = GyY_offset_sum/1000;
  GyZ_offset = GyZ_offset_sum/1000;
  
  AcX_offset = AcX_offset_sum/1000;
  AcY_offset = AcY_offset_sum/1000;
  AcZ_offset = AcZ_offset_sum/1000;
}

void angle_calc() {  //determine the intertial angle of the cubit in the X and Y axis

  MPU6050_read_gyros();
  MPU6050_read_accelerometers();
  MPU6050_filter_sensors();
}

void MPU6050_read_gyros()
{
    // read raw accel/gyro measurements from device - check the register map pg 31
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);   //dont know why 6 bytes are being requested
  GyX = ((float)(Wire.read() << 8 | Wire.read())/131)-GyX_offset; // 0x43 (GYRO_XOUT_H) bitwise-or 0x44 (GYRO_XOUT_L)
  GyY = ((float)(Wire.read() << 8 | Wire.read())/131)-GyY_offset; // 0x45 (GYRO_YOUT_H) bitwise-or 0x46 (GYRO_YOUT_L)
  GyZ = ((float)(Wire.read() << 8 | Wire.read())/131)-GyZ_offset; // 0x47 (GYRO_ZOUT_H) bitwise-or 0x48 (GYRO_ZOUT_L)
}

void MPU6050_read_accelerometers()
{
    Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  AcX = ((float)(Wire.read() << 8 | Wire.read())/16384)-AcX_offset; // 0x3B (ACCEL_XOUT_H) bitwise-or 0x3C (ACCEL_XOUT_L)
  AcY = ((float)(Wire.read() << 8 | Wire.read())/16384)-AcY_offset; // 0x3D (ACCEL_YOUT_H) bitwise-or 0x3E (ACCEL_YOUT_L)
  AcZ = ((float)(Wire.read() << 8 | Wire.read())/16384)-AcZ_offset; // 0x3F (ACCEL_ZOUT_H) bitwise-or 0x40 (ACCEL_ZOUT_L)
}

void MPU6050_filter_sensors()     //gyrofilt applies the LSB accuracy and makes the readings more stable
{    
    filter.updateIMU(GyX, GyY, GyZ, AcX, AcY, AcZ);
    theta1_X = filter.getRollRadians();
    theta1dot_X = GyX*0.01740;  
}