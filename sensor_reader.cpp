#include "sensor_reader.h"

#include <math.h>

namespace {

/// Defines the rotation matrix between the sensor frame and the robot frame.
const BLA::Matrix<3, 3> kImuToRobot = {1.0, 0.0, 0.0,
                                       0.0, 1.0, 0.0,
                                       0.0, 0.0, 1.0};

/**
 * @brief Defines the initial state for the Kalman filter. The first
 *   element is the angle and the second is the angular velocity.
 */
const BLA::Matrix<2> kInitialState = {0.0, 0.0};
/**
 * @brief Initial model covariance.
 */
const BLA::Matrix<2, 2> kInitialModelCov = {0.0001, 0.0,
                                            0.0, 0.001};
/**
 * @brief Initial observation covariance. These values are
 *   estimated from many sensor readings with the sensor
 *   sitting on the ground.
 */
const BLA::Matrix<2, 2> kInitialObservationCov = {0.15, 0.0,
                                                  0.0, 0.0001};

/// Time evolution matrix for the Kalman filter.
const BLA::Matrix<2, 2> kTimeEvolution = {1.0, 1.0,
                                          0.0, 1.0};
/// Observation matrix for the Kalman filter.
const BLA::Matrix<2, 2> kObservationMatrix = {1.0, 0.0,
                                              0.0, 1.0};

}  // namespace

SensorReader::SensorReader(uint32_t run_period) : run_period_(run_period) {}

void SensorReader::Begin() {
  // Try to initialize the sensor.
  while (!mpu_.begin()) {
    Serial.println("Failed to initialize sensor.");
    delay(1000);
  }

  mpu_.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu_.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize the Kalman filter.
  filter_.x = kInitialState;
  filter_.Q = kInitialModelCov;
  filter_.R = kInitialObservationCov;
  filter_.F = kTimeEvolution;
  filter_.H = kObservationMatrix;
}

void SensorReader::ReadAngle(float *angle, float *velocity) {
  // Update with the latest sensor readings.
  GetSensorReadings();

  // Get the observation vector.
  const BLA::Matrix<2> kObservations = {
    AngleFromAccel(),
    VelocityFromGyro()
  };
  // Update the filter with the measurements.
  filter_.update(kObservations);

  *angle = filter_.x(0);
  *velocity = filter_.x(1);
}

void SensorReader::GetSensorReadings() {
  mpu_.getEvent(&acceleration_, &gyro_, &temperature_);
}

float SensorReader::AngleFromAccel() {
  // Convert to the robot frame.
  const BLA::Matrix<3> kAccel = {
    acceleration_.acceleration.x, 
    acceleration_.acceleration.y,
    acceleration_.acceleration.z,
  };
  Multiply(kImuToRobot, kAccel, accel_robot_);

  // In the robot frame, the z-axis points down, the
  // y-axis points right, and the x-axis points outwards.
  const float kAccelDown = kAccel(2);
  const float kAccelOut = -kAccel(0);
  return atan2(kAccelOut, kAccelDown);
}

float SensorReader::VelocityFromGyro() {
  float velocity_mag = sqrt(
    pow(gyro_.gyro.x, 2.0) +
    pow(gyro_.gyro.y, 2.0) +
    pow(gyro_.gyro.z, 2.0)
  );

  if (gyro_.gyro.y < 0) {
    // Direction is negative.
    velocity_mag = -velocity_mag;
  }

  // Convert the velocity to rad/cycle.
  return velocity_mag * run_period_ / 1000;
}
