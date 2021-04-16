#include "sensor_reader.h"

#include <math.h>

namespace {

/// Defines the rotation matrix between the sensor frame and the robot frame.
const BLA::Matrix<3, 3> kImuToRobot = {1.0, 0.0, 0.0,
                                       0.0, -1.0, 0.0,
                                       0.0, 0.0, 1.0};

/**
 * @brief Defines the initial state for the Kalman filter. The first
 *   element is the angle and the second is the angular velocity.
 */
const BLA::Matrix<2> kInitialState = {0.0, 0.0};
/**
 * @brief Initial model covariance.
 */
const BLA::Matrix<2, 2> kInitialModelCov = {0.001, 0.0,
                                            0.0, 0.001};
/**
 * @brief Initial observation covariance. These values are
 *   estimated from many sensor readings with the sensor
 *   sitting on the ground.
 */
const BLA::Matrix<2, 2> kInitialObservationCov = {0.000438, 0.0,
                                                  0.0, 0.0};

/// Time evolution matrix for the Kalman filter.
const BLA::Matrix<2, 2> kTimeEvolution = {1.0, 1.0,
                                          0.0, 1.0};
/// Observation matrix for the Kalman filter.
const BLA::Matrix<2, 2> kObservationMatrix = {1.0, 0.0,
                                              0.0, 1.0};

}  // namespace

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

float SensorReader::ReadAngle() {
  // Update with the latest sensor readings.
  GetSensorReadings();

  // Get the observation vector.
  const BLA::Matrix<2> kObservations = {
    AngleFromAccel(),
    VelocityFromGyro()
  };
  // Update the filter with the measurements.
  filter_.update(kObservations);

  return filter_.x(0);
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

  // In the robot frame, the y-axis points up, the
  // z-axis points outwards, and the x-axis points right.
  const float kAccelDown = -kAccel(1);
  const float kAccelOut = -kAccel(0);
  return atan2(kAccelOut, kAccelDown);
}

float SensorReader::VelocityFromGyro() {
  return sqrt(
    pow(gyro_.gyro.x, 2.0) +
    pow(gyro_.gyro.y, 2.0) +
    pow(gyro_.gyro.z, 2.0)
  );
}
