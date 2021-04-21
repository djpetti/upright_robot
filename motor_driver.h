#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

/**
 * @brief Handles driving a motor connected to an H-bridge.
 */
class MotorDriver {
 public:
  /**
   * @param output_pin_1 The pin number that we will use as the first PWM output.
   * @param output_pin_2 The pin number that we will use as the second PWM output.
   */
  MotorDriver(int output_pin_1, int output_pin_2);

  /**
   * @brief Initializes the motor pins.
   */
  void Begin();

  /**
   * @brief Sets the speed of the motor.
   * @param speed The speed, from -255 to 255. Zero is stopped, and negative
   *   values move in the reverse direction.
   */
  void SetSpeed(int speed);

 private:
  /// The PWM output pins
  int pwm_pin_1_;
  int pwm_pin_2_;
};

#endif  // MOTOR_DRIVER_H_
