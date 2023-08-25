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
void Set_pwm() {

  req_torqueX =  K1 * theta1_X;
  req_torqueX += K2 * theta1dot_X;
  req_torqueX += K3 * theta2dot_X; 

  req_accX = ((req_torqueX*((currentT - previousT_1)))/((1.877e-3)*1000));
 if (abs(req_accX) > 0.1)
 {
    theta2dot_X = constrain((req_accX + theta2dot_X),-(max_spd),max_spd);
 }

  req_torqueY =  K1 * theta1_Y;
  req_torqueY += K2 * theta1dot_Y;
  req_torqueY += K3 * theta2dot_Y; 

  req_accY = ((req_torqueY*((currentT - previousT_1)))/((1.877e-3)*1000));
 if (abs(req_accY) > 0.1)
 {
    theta2dot_Y = constrain((req_accY + theta2dot_Y),-(max_spd),max_spd);
 }


}
void Motor_set_speed()
{
  m1 = constrain((0.6*theta2dot_X - theta2dot_Y/0.866),-max_spd,max_spd); 
  m2 = constrain((0.6*theta2dot_X + theta2dot_Y/0.866),-max_spd,max_spd);
  m3 = constrain(-theta2dot_X,-max_spd,max_spd);

  Motor_control(1,m1);
  Motor_control(2,m2);
  Motor_control(3,m3);
}
void Tune()
{
  if(Serial.available())
  {TxRx = Serial.read();}
  switch(TxRx)
  {
    case 'a':
    K1  = K1+0.1;
    break;

    case 's':
    K1 = K1-0.1;
    break;

    case 'd':
    K2 = K2+0.1;
    break;

    case 'f':
    K2 = K2-0.1;
    break;

    case 'g':
    K3 = K3+0.001;
    break;

    case 'h':
    K3 = K3-0.001;
    break;

  }
TxRx = '0';
}

void PrintData()
{
  Serial.print(theta1_X);
  Serial.print("  ");
  Serial.print(theta1_Y);
  Serial.print("  ");
  Serial.print(m1);
  Serial.print("  ");
  Serial.print(m2);
  Serial.print("  ");
  Serial.println(m3);
}