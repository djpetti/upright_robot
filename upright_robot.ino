#include "controller.h"
#include "motor_driver.h"
#include "rate_limit.h"
#include "sensor_reader.h"

namespace {

/// PID gains.
const int kP = 1600;
const int kD = 5500;

/// PWM output for the motors.
const int kLeftMotorPwm1 = 3;
const int kLeftMotorPwm2 = 2;
const int kRightMotorPwm1 = 5;
const int kRightMotorPwm2 = 4;

/// Pin that the trim pot is connected to.
const int kTrimPotPin = A1;

/// Period at which we read the sensors, in ms.
const int kSensorReadPeriod = 10;
/**
 * @brief Factor by which we multiply the sensor read
 *  period in order to get the controller period.
 */
const int kLoopPeriodMultiplier = 3;
const int kLoopPeriod = kSensorReadPeriod * kLoopPeriodMultiplier;

/// Angle setpoint, in radians.
const float kGoalAngle = 0.0;

/// Minimum PWM output for deadband compensation.
const uint8_t kDeadbandPwm = 5;

/// PD controller.
Controller g_controller(kP, kD, kLoopPeriod);

/// Reads filtered measurements from the sensor.
SensorReader* g_sensor_reader;

/// Handles motor output.
MotorDriver g_left_motor_driver(kLeftMotorPwm1, kLeftMotorPwm2);
MotorDriver g_right_motor_driver(kRightMotorPwm1, kRightMotorPwm2);

/// Stores the trim amount.
float g_trim = 0.0;

/// Used for limiting loop rate.
RateLimit g_rate_limit(kSensorReadPeriod);
/// Number of iterations since we've run the controller.
int g_ticks_since_control_update = 0;

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

  g_sensor_reader = new SensorReader(kSensorReadPeriod);
  g_sensor_reader->Begin();

  g_left_motor_driver.Begin();
  g_right_motor_driver.Begin();
  
  // Allow the motor to drive forwards and backwards.
  g_controller.SetOutputLimits(-255, 255);
  g_controller.SetDeadbandCompensation(kDeadbandPwm);
  g_controller.SetGoal(kGoalAngle);
}

void loop() {
  // Read latest from the sensor.
  float angle, velocity;
  g_sensor_reader->ReadAngle(&angle, &velocity);
  angle += g_trim;

  if (++g_ticks_since_control_update >= kLoopPeriodMultiplier) {
    // Update the controller.
    const int kOutput = g_controller.ComputeOutput(angle, velocity);
    
    // Write to the output. Since it's differential drive,
    // the motors have to turn in opposite directions.
    g_left_motor_driver.SetSpeed(-kOutput);
    g_right_motor_driver.SetSpeed(kOutput);

    g_ticks_since_control_update = 0;
  }

  g_rate_limit.Limit();
}
