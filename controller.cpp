#include "controller.h"

#include <algorithm>
#include <cstdint>
#include <Arduino.h>

namespace {

/// Multiplication factor to use when converting floats to ints.
constexpr int kFixedPrecision = 100000;
  
}  // namespace

Controller::Controller(int p_gain, int d_gain, int run_period) 
  : p_gain_(p_gain), d_gain_(d_gain), period_(run_period) {}

void Controller::SetOutputLimits(int output_min, int output_max) {
  min_output_ = output_min;
  max_output_ = output_max;
}

void Controller::SetDeadbandCompensation(int compensation) {
  deadband_ = compensation;
}

void Controller::SetGoal(float goal) {
  // Convert to an integer with fixed precision.
  goal_ = static_cast<int>(goal * kFixedPrecision);
}

int Controller::ComputeOutput(float measurement) {
  // Convert to an integer with fixed precision.
  measurement = static_cast<int>(measurement * kFixedPrecision);
  // Calculate error.
  const int kError = goal_ - measurement;

  // Apply the gains.
  const int kDerivative = SmoothDerivative(kError);
  int output = p_gain_ * kError + d_gain_ * kDerivative;

  // Rescale back to the input scale.
  output /= kFixedPrecision;
  // Limit and apply deadband compensation.
  output = CompensateDeadband(output);
  return LimitOutput(output);
}

int Controller::SmoothDerivative(int next_error) {
  // Update the previous errors.
  for (uint32_t i = previous_errors_.size() - 1; i > 0; --i) {
    previous_errors_[i] = previous_errors_[i - 1];
  }
  previous_errors_[0] = next_error;

  // Calculate the derivative.
  const int kDerivative = previous_errors_[0] + previous_errors_[1] * 2
    - previous_errors_[3] * 2 - previous_errors_[4];
  return kDerivative / (period_ * 8); 
}

int Controller::LimitOutput(int output) {
  output = std::max(min_output_, output);
  output = std::min(max_output_, output);

  return output;
}

int Controller::CompensateDeadband(int output) {
  if (output > 0)  {
    return output + deadband_;
  } else {
    return output - deadband_;
  }
}
