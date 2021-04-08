#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

/**
 * @brief Handles driving a motor connected to an H-bridge.
 */
class MotorDriver {
 public:
  /**
   * @param pwm_pin The pin number that we will use as the PWM output.
   * @param direction_pin The pin number that we will use to set the direction.
   */
  MotorDriver(int pwm_pin, int direction_pin);  

  /**
   * @brief Sets the speed of the motor.
   * @param speed The speed, from -255 to 255. Zero is stopped, and negative
   *   values move in the reverse direction.
   */
  void SetSpeed(int speed);

 private:
  /// The PWM output pin.
  int pwm_pin_;
  /// The direction output pin.
  int direction_pin_;
};

#endif  // MOTOR_DRIVER_H_
