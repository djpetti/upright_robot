#include <PID_v1.h>

#include "motor_driver.h"
#include "sensor_reader.h"

namespace {

/// PID gains.
const float kP = 900.0;
const float kD = 0.0;
// Since this is a PD controller, I-gain is zero.
const float kI = 0.0;

/// PWM output for the motors.
const int kLeftMotorPwm1 = 3;
const int kLeftMotorPwm2 = 2;
const int kRightMotorPwm1 = 5;
const int kRightMotorPwm2 = 4;

/// Pin that the trim pot is connected to.
const int kTrimPotPin = A1;

/// Loop period, in ms.
const int kLoopPeriod = 10;

/// Current angle measurement.
double g_angle = 0.0;
/// Current motor PWM output.
double g_motor_pwm = 0.0;
/// Angle setpoint, in radians.
double g_goal_angle = 0.0;

/// Minimum PWM output for deadband compensation.
const uint8_t kDeadbandPwm = 0;

/// PID controller.
PID g_pid(&g_angle, &g_motor_pwm, &g_goal_angle, kP, kI, kD, DIRECT);

/// Reads filtered measurements from the sensor.
SensorReader* g_sensor_reader;

/// Handles motor output.
MotorDriver g_left_motor_driver(kLeftMotorPwm1, kLeftMotorPwm2);
MotorDriver g_right_motor_driver(kRightMotorPwm1, kRightMotorPwm2);

/// Stores the trim amount.
float g_trim = 0.0;

/**
 * @brief Reads the trim to set on the IMU. 
 * @return The trim offset to add.
 */
float GetTrim() {
  // Read the potentiometer.
  const int pot_value = analogRead(kTrimPotPin);
  const uint32_t max_adc_value = 1 << 12;
  return (float)(pot_value - (int)(max_adc_value / 2)) / max_adc_value;
}

}  // namespace

void setup() {
  // Initialize the serial.
  Serial.begin(9600);

  // Enable 12-bit ADC.
  analogReadResolution(12);

  // Read the trim setting.
  g_trim = GetTrim();

  g_sensor_reader = new SensorReader(kLoopPeriod);
  g_sensor_reader->Begin();

  g_left_motor_driver.Begin();
  g_right_motor_driver.Begin();
  
  // Allow the motor to drive forwards and backwards.
  g_pid.SetOutputLimits(-255.0, 255.0);
  // Turn on the controller.
  g_pid.SetMode(AUTOMATIC);
}

void loop() {
  // Read latest from the sensor.
  g_angle = g_sensor_reader->ReadAngle() + g_trim;
  // Update the controller.
  g_pid.Compute();

  // Deadband compensation.
  if (g_motor_pwm > 0) {
    g_motor_pwm += kDeadbandPwm;
  } else {
    g_motor_pwm -= kDeadbandPwm;
  }

  g_motor_pwm = min(255, g_motor_pwm);
  g_motor_pwm = max(-255, g_motor_pwm);
  
  // Write to the output. Since it's differential drive,
  // the motors have to turn in opposite directions.
  g_left_motor_driver.SetSpeed(-g_motor_pwm);
  g_right_motor_driver.SetSpeed(g_motor_pwm);

  delay(kLoopPeriod);
}
