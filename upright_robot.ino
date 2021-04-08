#include <PID_v1.h>

#include "motor_driver.h"
#include "sensor_reader.h"

namespace {

/// PID gains.
const float kP = 10.0;
const float kD = 1.0;
// Since this is a PD controller, I-gain is zero.
const float kI = 0.0;

/// PWM output for left motor.
const int kLeftMotorPWMPin = 2;
/// PWM output for right motor.
const int kRightMotorPWMPin = 3;
/// Direction pin for the left motor.
const int kLeftMotorDirPin = 0;
/// Direction pin for the right motor.
const int kRightMotorDirPin = 1;

/// Loop period, in ms.
const int kLoopPeriod = 10;

/// Current angle measurement.
double g_angle = 0.0;
/// Current motor PWM output.
double g_motor_pwm = 0.0;
/// Angle setpoint, in radians.
double g_goal_angle = 0.0;

/// PID controller.
PID g_pid(&g_angle, &g_motor_pwm, &g_goal_angle, kP, kI, kD, DIRECT);

/// Reads filtered measurements from the sensor.
SensorReader g_sensor_reader;

/// Handles motor output.
MotorDriver g_left_motor_driver(kLeftMotorPWMPin, kLeftMotorDirPin);
MotorDriver g_right_motor_driver(kRightMotorPWMPin, kRightMotorDirPin);


}  // namespace

void setup() {
  // Allow the motor to drive forwards and backwards.
  g_pid.SetOutputLimits(-255.0, 255.0);
  // Turn on the controller.
  g_pid.SetMode(AUTOMATIC);
}

void loop() {
  // Read latest from the sensor.
  g_angle = g_sensor_reader.ReadAngle();
  // Update the controller.
  g_pid.Compute();
  
  // Write to the output.
  g_left_motor_driver.SetSpeed(g_motor_pwm);
  g_right_motor_driver.SetSpeed(g_motor_pwm);

  delay(kLoopPeriod);
}
