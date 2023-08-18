
//test to see if control strategy works for one dimentional harware
//dirk Slabber
//12/08/2023

#include "arduino_cube.h"
#include <stdint.h>
#include <Wire.h>
#include <EEPROM.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  TCCR1A = 0b00000001; 
  TCCR1B = 0b00001010;
  TCCR2B = 0b00000010;
  TCCR2A = 0b00000011;

  pinMode(DIR_1, OUTPUT);     //sets motor diretion outputs and other digital pins
  pinMode(11, OUTPUT);


  pinMode(DIR_2, OUTPUT);
  pinMode(DIR_3, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(BUZZER, OUTPUT);
 
  Motor_control(0);
  Calc_sys_param();

  MPU6050_init();
  MPU6050_offset_setup();
  angle_calc();
  
  digitalWrite(BRAKE, HIGH);
}

void loop() {
  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {
    angle_calc(); 
    Set_pwm();
    Motor_control(-theta2dot_1);
    PrintData();
    previousT_1 = currentT;
    }
  }

