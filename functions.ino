

void writeTo(byte device, byte address, byte value) {  // simplifies writing to registers
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void Calc_sys_param()
{
float I_wc = (M*0.137*0.137);         //reaction wheel inertia (kgm2)
float I_pb = m*l*l;                 //pendulum inertia (kgm2)
float max_acc= max_torque/(I_wc);   //assumed based on power (hence the 2)
}

void MPU6050_offset_setup() {  //, scaling and ......

  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    angleX_offset_sum += Acc_angleX;
    delay(1);
  }
  GyX_offset = GyX_offset_sum/1024;
  angleX_offset = angleX_offset_sum/1024;
}

void angle_calc() {  //determine the intertial angle of the cubit in the X and Y axis

  MPU6050_read_gyros();
  MPU6050_read_accelerometers();
  MPU6050_filter_sensors();

  Gyr_angleX = ((GyXF) * loop_time )*0.01740*0.001; // incremental increase in angle based on gyroscope
  Acc_angleX =(atan2(AcYF, AcZF)-angleX_offset);
  theta1_X += (Gyr_angleX*Gyro_amount);  //angle as increased
  theta1_X += (Acc_angleX * (1.0 - Gyro_amount));
  theta1_X = constrain(theta1_X,-abs(Acc_angleX),abs(Acc_angleX));


  theta1dot_X = GyXF*0.01740;
}

void Motor_control(int sp) {
  sp = (sp*255)/max_spd;
  sp = constrain(sp,-255,255);
      if (sp > 0) digitalWrite(DIR_1, LOW);
      else digitalWrite(DIR_1, HIGH);
      analogWrite(PWM_1, 255 - abs(sp));
}

void Set_pwm() {

  req_torque =  K1 * theta1_X;
  req_torque += K2 * theta1dot_X;
  req_torque += K3 * theta2dot_1; 

  req_acc = ((req_torque*(loop_time))/((1.877e-3)*1000))*(K4);

  theta2dot_1 = constrain((req_acc + theta2dot_1),-(max_spd),max_spd);
}
void PrintData()
{
    Serial.print(theta1_X);
    Serial.print(",");
    Serial.print(theta1dot_X);
    Serial.print(",");
    Serial.print(req_acc);
    Serial.print(",");
    Serial.print(theta2dot_1);
    Serial.println("");
}