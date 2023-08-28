// simplifies writing to I2C devides
void writeTo(byte device, byte address, byte value) 
{ 
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void Beep()
{
  digitalWrite(BUZZER, 1);
  delay(100);
  digitalWrite(BUZZER, 0);
}

void Motor_control(int M, float sp) //accepts sp in radians
{
  int pwm,dir;
  switch (M)
  {
    case 1:
    pwm = PWM_1;
    dir = DIR_1;
    break;
    case 2:
    pwm = PWM_2;
    dir = DIR_2;
    break;
    case 3:
    pwm = PWM_3;
    dir = DIR_3;
    break;
  }
  sp = (sp*255)/max_spd;
      if (sp > 0) digitalWrite(dir, LOW);
      else digitalWrite(dir, HIGH);
      analogWrite(pwm, 255 - abs(sp));
}
void motor_demo()
{
  float sp =0;

  for (int i=0;i<=255;i++)
  {
    sp = (i*max_spd)/255;
    Motor_control(1,(sp));
    Motor_control(2,(sp));
    Motor_control(3,(sp));
    Serial.println(sp);
    delay(10);
  }
    for (int i=255;i>=0;i--)
  {
    sp = (i*max_spd)/255;
    Motor_control(1,(sp));
    Motor_control(2,(sp));
    Motor_control(3,(sp));
    Serial.println(sp);
    delay(10);
  }
}
void Set_pwm() { // assume a triangle with the motors perfectly horizontal the speeds are calculated thus
 
  req_torque1 =  K1 * (theta1_X/(3*0.5) - theta1_Y/(2*0.866));
  req_torque1 += K2 * (theta1dot_X/(3*0.5) - theta1dot_Y/(2*0.866));
  req_torque1 += K3 * theta2dot_1; 
  req_acc1 = ((req_torque1*((currentT - previousT_1)))/((1.877e-3)*1000));
  theta2dot_1 = constrain((req_acc1/0.8165 + theta2dot_1),-(max_spd),max_spd);

  req_torque2 =  K1 * (theta1_X/(3*0.5) + theta1_Y/(2*0.866));
  req_torque2 += K2 * (theta1dot_X/(3*0.5) + theta1dot_Y/(2*0.866));
  req_torque2 += K3 * theta2dot_2; 
  req_acc2 = ((req_torque2*((currentT - previousT_1)))/((1.877e-3)*1000));
  theta2dot_2 = constrain((req_acc2/0.8165 + theta2dot_2),-(max_spd),max_spd);

  req_torque3 =  K1 * (theta1_X/3);
  req_torque3 += K2 * theta1dot_X/3;
  req_torque3 += K3 * theta2dot_3; 
  req_acc3 = ((req_torque3*((currentT - previousT_1)))/((1.877e-3)*1000));
  theta2dot_3 = constrain((req_acc3/0.8165 + theta2dot_3),-(max_spd),max_spd);

}
void Motor_set_speed()
{
  Motor_control(1,theta2dot_1);
  Motor_control(2,theta2dot_2);
  Motor_control(3,-theta2dot_3);
}


void PrintData()
{
  Serial.print(K1);
  Serial.print("  ");
  Serial.print(K2);
  Serial.print("  ");
  Serial.println(K3,4);
}

void process_commands() {
  if(hc06.available()) {
    TxRx = hc06.read();
    Serial.println(TxRx);
  }
  // if (Serial.available())
  // {
  //   TxRx = Serial.read();
  // }

  switch(TxRx) {
  case 'a':
    K1  = K1+1;
    break;

  case 's':
    K1 = K1-1;
    break;

  case 'd':
    K2 = K2+0.1;
    break;

  case 'f':
    K2 = K2-0.1;
    break;
  case 'g':
    K3 = K3+0.0005;
    break;

  case 'h':
    K3 = K3-0.0005;
    break;

  case '0':
    digitalWrite(13, LOW);
    break;

  case '1':
    digitalWrite(13, HIGH);
    break;

  default:
    // Serial.println("Unknown command!");
    break;
  }
  TxRx = ' ';
}

void send_telemetry() {

  // byte *theta1_X_data = &theta1_X;

  hc06.print("theta1_X: ");
  hc06.print(theta1_X);
  hc06.print("    theta1_Y: ");
  hc06.print(theta1_Y);
  hc06.println();
}
