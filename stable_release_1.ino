#include <MadgwickAHRS.h>
#include "arduino_cube.h"
#include <stdint.h>
#include <Wire.h>
//dirk Slabber
//24/08/2023

void setup() {
  Serial.begin(115200);
  //interrupt timers
  TCCR1A = 0b00000001; 
  TCCR1B = 0b00001010;
  TCCR2B = 0b00000010;
  TCCR2A = 0b00000011;
  //pinmodes
  pinMode(DIR_1, OUTPUT);     
  pinMode(PWM_1, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  //enable madgwick filter with a sample frequency of 25Hz
  filter.begin(mgwk_freq);

  //Ensure the motor starts at 0 rpm
  Motor_control(0);
  digitalWrite(BRAKE, HIGH);

  //Beeps to warn user that calibration is about to start
  delay(1000);
  Beep();
  delay(1000);
  Beep();

  //initialise the IMU and complete the offset calibration
  MPU6050_init();
  MPU6050_offset_setup();

  //set up filter interrupt interrupt
  millis_per_filter_reading = 1000/mgwk_freq;
  previousT_1 = millis();

  //beep a third time to announce start
  Beep();

}

void loop() {
  currentT = millis();
  //sample at the correct rate
  if ((currentT-previousT_1)>=millis_per_filter_reading)
  {
    angle_calc();
  }
  //
  if ((currentT - previousT_1) >= loop_time) 
  {
    Set_pwm();
    Motor_control(-theta2dot_1);
    Tune();
    PrintData();
    previousT_1 = currentT;
  }

}
