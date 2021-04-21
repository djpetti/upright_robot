#include "motor_driver.h"

#include <Arduino.h>
#include <math.h>

MotorDriver::MotorDriver(int output_pin_1, int output_pin_2) : 
  pwm_pin_1_(output_pin_1),
  pwm_pin_2_(output_pin_2) {
}

void MotorDriver::Begin() {
  // Set both pins as output.
  pinMode(pwm_pin_1_, OUTPUT);
  pinMode(pwm_pin_2_, OUTPUT);
}

void MotorDriver::SetSpeed(int speed) {
  // Set the direction.
  int low_pin = pwm_pin_2_;
  int high_pin = pwm_pin_1_;
  if (speed < 0) {
    low_pin = pwm_pin_1_;
    high_pin = pwm_pin_2_;
  }
  digitalWrite(low_pin, LOW);

  // Set the PWM duty cycle.
  analogWrite(high_pin, abs(speed));
}
