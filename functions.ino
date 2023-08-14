

void writeTo(byte device, byte address, byte value) {  // simplifies writing to registers
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void MPU6050_offset_setup() {  //, scaling and ......

  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    GyY_offset_sum += GyY;
    GyZ_offset_sum += GyZ;
    AcX_offset_sum += AcX;
    AcY_offset_sum += AcY;
    AcZ_offset_sum += AcZ;
    delay(1);
  }
  GyX_offset = GyX_offset_sum >> 10;
  GyY_offset = GyY_offset_sum >> 10;
  GyZ_offset = GyZ_offset_sum >> 10;



  AcX_offset = AcX_offset_sum >> 10;
  AcY_offset = AcY_offset_sum >> 10;
  AcZ_offset = AcZ_offset_sum >> 10;
}

void angle_calc() {  //determine the intertial angle of the cubit in the X and Y axis

  MPU6050_read_gyros();
  MPU6050_read_accelerometers();

  robot_angleX += GyX * loop_time / 1000 / 65.536;
  robot_angleY += GyY * loop_time / 1000 / 65.536;

  Acc_angleX = (atan2(AcY, AcZ)-atan2(AcY_offset,AcZ_offset)) * 57.2958;
  Acc_angleY = (-atan2(AcX, AcZ)+atan2(AcX_offset,AcZ_offset)) * 57.2958;

  robot_angleX = robot_angleX * (Gyro_amount) + Acc_angleX * (1.0 - Gyro_amount);  // somewhat unneccessary // i think he meant for these two to be swapped
  robot_angleY = robot_angleY * (Gyro_amount) + Acc_angleY * (1.0 - Gyro_amount);

  angleX = robot_angleX;
  angleY = robot_angleY;
}

void get_state_variables()
{
  theta1_X = angleX*0.017453;
  theta1_Y = angleY*0.017453;

  theta1dot_X = (float)gyroXfilt*0.01745;
  theta1dot_Y = (float)gyroYfilt*0.01745;
  theta1dot_Z = (float)gyroZfilt*0.01745;


}


void Motor_control(int sp, int M) {

  sp = constrain(sp,-255,255);
  theta2dot_1 = (sp*160)/255*0.1047;
  switch (M) {
    case 1:
      if (sp > 0) digitalWrite(DIR_1, LOW);
      else digitalWrite(DIR_1, HIGH);
      analogWrite(PWM_1, 255 - abs(sp));
      break;
    case 2:
      if (sp > 0) digitalWrite(DIR_2, LOW);
      break;
    case 3:
      if (sp > 0) digitalWrite(DIR_3, LOW);
      else digitalWrite(DIR_3, HIGH);
      analogWrite(PWM_3, 255 - abs(sp));
      break;
    default:
      break;
  }
}

void Set_pwm() {

  mot_V_X += K1 * theta1_X;
  mot_V_X += K2 * theta1dot_X;
  mot_V_X += K3 * theta2dot_2; 

  mot_V_Y += K1 * theta1_Y;
  mot_V_Y += K2 * theta1dot_Y;
  mot_V_Y += K3 * theta2dot_1;

  mot_V_Y -= K4 * ((mot_V_Y));

  mot_V_X = constrain(mot_V_X,-12,12);
  mot_V_Y = constrain(mot_V_Y,-12,12);
  


  pwm_X = ((mot_V_X*255)/12); 
  pwm_Y = ((mot_V_Y*255)/12);
}
void PrintData()
{
    Serial.print(theta1_Y*K1);
    Serial.print(",");
    Serial.print(theta1dot_Y*K2);
    Serial.print(",");
    Serial.print(theta2dot_1*K3);
    Serial.print(",");
    Serial.print(mot_V_Y);
    Serial.println("");
}