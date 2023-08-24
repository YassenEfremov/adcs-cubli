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

void Motor_control(int sp) //accepts sp in radians
{
  sp = (sp*255)/max_spd;
      if (sp > 0) digitalWrite(DIR_1, LOW);
      else digitalWrite(DIR_1, HIGH);
      analogWrite(PWM_1, 255 - abs(sp));
}

void Set_pwm() {

  req_torque =  K1 * theta1_X;
  req_torque += K2 * theta1dot_X;
  req_torque += K3 * theta2dot_1; 

  req_acc = ((req_torque*((currentT - previousT_1)))/((1.877e-3)*1000));

 if (abs(req_acc) > 0.1)
 {
    theta2dot_1 = constrain((req_acc + theta2dot_1),-(max_spd),max_spd);
 }
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
  Serial.print(K1);
  Serial.print("  ");
  Serial.print(K2);
  Serial.print("  ");
  Serial.print(K3);
  Serial.print("  ");
  Serial.print(theta1_X);
  Serial.print("  ");
  Serial.print(theta1dot_X);
  Serial.print("  ");
  Serial.println(theta2dot_1);
}