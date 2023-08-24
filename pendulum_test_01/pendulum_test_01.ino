
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
 
  Motor_control(0, 1);
  Motor_control(0, 2);
  Motor_control(0, 3);

  MPU6050_init();
  MPU6050_offset_setup();
  digitalWrite(BRAKE, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {
    angle_calc(); 
    MPU6050_filter_gyros();
    get_state_variables();
    Set_pwm();
    Motor_control(pwm_Y,1);
    PrintData();
    previousT_1 = currentT;
    }

    angleX = 0;
  }

