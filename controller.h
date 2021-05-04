#include <array>
#include <limits>

/**
 * @brief Implements the controller for the robot.
 */
class Controller {
 public:
  /**
   * @param p_gain The proportional gain.
   * @param d_gain The derivative gain.
   * @param run_period The period at which the controller will be run, in ms.
   */
  Controller(int p_gain, int d_gain, int run_period);

  /**
   * @brief Sets the maximum and minimum values for the output.
   * @param output_min The minimum output value.
   * @param output_max The maximum output value.
   */
  void SetOutputLimits(int output_min, int output_max);

  /**
   * @brief Sets the value for deadband compenstation.
   * @param The amount of compensation to use.
   */
  void SetDeadbandCompensation(int compensation);

  /**
   * @brief Sets the desired position.
   * @param goal The goal to set.
   */
  void SetGoal(float goal);

  /**
   * @brief Computes the output to use.
   * @return The computed output.
   */
  int ComputeOutput(float measurement);

 private:
  /**
   * @brief Calculates a smoothed derivative value.
   * @param next_error The most recent error measurement.
   * @return The smoothed error derivative.
   */
  int SmoothDerivative(int next_error);

  /**
   * @param The raw output.
   * @return The output clipped according to the set limits.
   */
  int LimitOutput(int output);

  /**
   * @brief Applies deadband compensation.
   * @param output The raw output.
   * @return The output with deadband compensation applied.
   */
  int CompensateDeadband(int output);
 
  /// The proportional gain.
  int p_gain_;
  /// The derivative gain.
  int d_gain_;
  /// The big-T value for the controller.
  int period_;

  /// The minimum output value.
  int min_output_ = std::numeric_limits<int>::min();
  /// The maximum output value.
  int max_output_ = std::numeric_limits<int>::max();

  /// The amount of deadband compensation.
  int deadband_ = 0;

  /// The specified goal.
  int goal_ = 0;
  /// Stores previous measurements for smooth derivative.
  std::array<int, 5> previous_errors_ = {0, 0, 0, 0, 0};
};
