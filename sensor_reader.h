#ifndef SENSOR_READER_H_
#define SENSOR_READER_H_

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BasicLinearAlgebra.h>
#include <Kalman.h>

/**
 * @brief Is responsible for reading filtered angles from the MPU6050
 */
class SensorReader {
 public:
  SensorReader();
 
  /**
   * @return The current tilt angle of the robot, in rads.
   */
  float ReadAngle(); 

 private:
  /**
   * @brief Updates the latest raw sensor readings.
   */
  void GetSensorReadings();

  /**
   * @brief Calculates an angle using just the accelerometer.
   * @return The angle that it calculated.
   */
  float AngleFromAccel();

  /**
   * @brief Gets the magnitude of the angular velocity using the gyro.
   * @return The angular velocity that it calculated.
   */
  float VelocityFromGyro();
 
  /// The sensor handle.
  Adafruit_MPU6050 mpu_; 
  /// Stores latest readings from the sensor.
  sensors_event_t acceleration_;
  sensors_event_t gyro_;
  sensors_event_t temperature_;

  /// Matrix to use for storing acceleration in robot frame.
  BLA::Matrix<3> accel_robot_;

  /// Kalman filter.
  KALMAN<2, 2> filter_;
};

#endif  // SENSOR_READER_H_
