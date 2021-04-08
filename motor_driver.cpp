#include "motor_driver.h"

#include <Arduino.h>
#include <math.h>

MotorDriver::MotorDriver(int pwm_pin, int direction_pin) : 
  pwm_pin_(pwm_pin),
  direction_pin_(direction_pin) {
  // Set both pins as output.
  pinMode(pwm_pin_, OUTPUT);
  pinMode(direction_pin_, OUTPUT);
}

void MotorDriver::SetSpeed(int speed) {
  // Set the direction.
  if (speed < 0) {
    digitalWrite(direction_pin_, HIGH);
  } else {
    digitalWrite(direction_pin_, LOW);
  }

  // Set the PWM duty cycle.
  analogWrite(pwm_pin_, abs(speed));
}
