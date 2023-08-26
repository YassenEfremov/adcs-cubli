#include <MadgwickAHRS.h>
#include "arduino_cube.h"
#include <stdint.h>
#include <Wire.h>
#include <SoftwareSerial.h>
//dirk Slabber
//24/08/2023

SoftwareSerial hc06(2, 3);

void setup() {
  Serial.begin(9600);
  hc06.begin(9600);
  //interrupt timers
  TCCR1A = 0b00000001; 
  TCCR1B = 0b00001010;
  TCCR2B = 0b00000010;
  TCCR2A = 0b00000011;
  //pinmodes
  pinMode(DIR_1, OUTPUT);     
  pinMode(PWM_1, OUTPUT);
  pinMode(DIR_2, OUTPUT);     
  pinMode(PWM_2, OUTPUT);
  pinMode(DIR_3, OUTPUT);     
  pinMode(PWM_3, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  //enable madgwick filter with a sample frequency of 25Hz
  filter.begin(mgwk_freq);

  //Ensure the motor starts at 0 rpm
  delay(100);
  Motor_control(1, 0);
  Motor_control(2, 0);
  Motor_control(3, 0);
  digitalWrite(BRAKE, HIGH);
  //motor_demo();
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
  
  //beep a third time to announce start
  Beep();

  previousT_1 = millis();
}

void loop() {
  currentT = millis();
  //sample at the correct rate
  if ((currentT-previousT_1)>=millis_per_filter_reading)
  {
    angle_calc();
  }
  //run the system actuation at correct rate
  if ((currentT - previousT_1) >= loop_time) 
  {
    Set_pwm();
    Motor_set_speed();
    Tune();
    // PrintData();
    previousT_1 = currentT;
  }

  // process_commands();

  // chat v v v

  // while (hc06.available() > 0) {
  //   char c = hc06.read();
  //   Serial.print(c);
  // }
  
  // while (Serial.available() > 0) {
  //   char c = Serial.read();
  //   hc06.write(c);
  //   Serial.print(c);
  // }

  // delay(10);
}
