#include "arduino_cube.h"

#include <Wire.h>
#include <AltSoftSerial.h>  // full-duplex! (SoftwareSerial.h is half-duplex!)
#include <stdint.h>
//dirk Slabber
//24/08/2023

AltSoftSerial hc06;


void setup() {
  Serial.begin(9600);
  hc06.begin(9600);
  //interrupt timers        // These interfere with the AltSoftSerial library!
  // TCCR1A = 0b00000001;   // (do we even need them?)
  // TCCR1B = 0b00001010;
  // TCCR2B = 0b00000010;
  // TCCR2A = 0b00000011;
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
  // delay(100);
  Motor_control(1, 0);
  Motor_control(2, 0);
  Motor_control(3, 0);
  //motor_demo();
  //beeps to warn user that calibration is about to start
  // delay(5000);
  // beep();
  // delay(5000);
  // beep();
  // delay(100);

  //initialise the IMU and complete the offset calibration
  MPU6050_init();
  delay(100);
  MPU6050_offset_setup();

  //set up filter interrupt interrupt
  millis_per_filter_reading = 1000/mgwk_freq;
  
  //beep a third time to announce start
  beep();

  previousT_1 = millis();
}


void loop() {

  currentT = millis();

  //sample at the correct rate
  if ((currentT - previousT_1) >= millis_per_filter_reading) {
    angle_calc();
  }

  //run the system actuation at correct rate
  if ((currentT - previousT_1) >= loop_time) {
    Set_pwm();
    Motor_set_speed();
    process_commands();
    // send_telemetry();
    // PrintData();
    previousT_1 = currentT;
  }
}
