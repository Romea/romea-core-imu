# romea_core_imu

## Overview

`romea_core_imu` is a C++ library for modelling IMU devices, creating typed IMU data frames and providing lightweight attitude and motion-detection algorithms.

The package is framework-independent C++ code. Middleware-specific drivers, messages and launch files are intentionally kept outside this library.

## IMU models

| Model | Class | Measurements |
| ----- | ----- | ------------ |
| 6-DOF IMU | `IMU6DOF` | 3-axis linear acceleration and 3-axis angular speed. |
| 9-DOF IMU | `IMU9DOF` | 6-DOF IMU data plus 3-axis magnetic field. |
| VRU | `IMUVRU` | 6-DOF IMU data plus roll and pitch estimates. |
| AHRS | `IMUAHRS` | 9-DOF IMU data plus roll, pitch and course estimates. |

The models are incremental: `IMU9DOF` extends `IMU6DOF` with magnetic measurements, while `IMUVRU` and `IMUAHRS` add attitude estimates provided by the device.

---

## Sensor characteristics

Each IMU model stores device-level information and per-channel measurement characteristics used by algorithms that consume inertial data:

| Scope | Quantities | Meaning |
| ----- | ---------- | ------- |
| Device | `rate`, `bodyPose` | Nominal update rate and rigid transform from the body frame to the IMU frame. |
| Linear acceleration | `accelerationNoiseDensity`, `accelerationBiasStabilityStd`, `accelerationRange` | Noise, bias stability and measurement range of the accelerometer channels. |
| Angular speed | `angularSpeedNoiseDensity`, `angularSpeedBiasStabilityStd`, `angularSpeedRange` | Noise, bias stability and measurement range of the gyroscope channels. |
| Magnetic field | `magneticNoiseDensity`, `magneticBiasStabilityStd`, `magneticRange` | Noise, bias stability and measurement range of the magnetometer channels, when available. |
| Attitude angles | `angleStd` | Standard deviation of roll, pitch or course estimates, when provided by the device. |

The classes also provide helper methods returning standard deviations and variances for the quantities they model.

---

## Data frames

| Frame | Content |
| ----- | ------- |
| `AccelerationsFrame` | Linear acceleration along the IMU x, y and z axes. |
| `AngularSpeedsFrame` | Angular speed around the IMU x, y and z axes. |
| `MagneticsFrame` | Magnetic field along the IMU x, y and z axes. |
| `RollPitchFrame` | Roll and pitch angles. |
| `RollPitchCourseFrame` | Roll, pitch and course angles. |

The frame structures are intentionally small. They are used by higher-level code to carry typed measurements without imposing a transport format.

---

## Algorithms

| Algorithm | Class | Purpose |
| --------- | ----- | ------- |
| TRIAD attitude estimation | `TriadAttitude` | Estimate attitude from accelerometer and magnetometer vectors using two non-collinear reference directions. |
| Zero-velocity detection | `ZeroVelocityEstimator` | Detect stationary periods from acceleration and angular speed variance over a moving time window. |
| Roll/pitch Kalman estimation | `RollPitchKalmanEstimator` | Estimate roll and pitch from accelerometer and gyroscope data. |

These algorithms are lightweight building blocks. They do not impose a complete inertial navigation pipeline; applications can combine them with their own calibration, filtering and diagnostic logic.

---

## Minimal usage

```cpp
#include <romea_core_imu/IMU6DOF.hpp>
#include <romea_core_imu/algorithms/RollPitchKalmanEstimator.hpp>

romea::core::IMU6DOF imu(
  rate,
  acceleration_noise_density,
  acceleration_bias_stability_std,
  acceleration_range,
  angular_speed_noise_density,
  angular_speed_bias_stability_std,
  angular_speed_range,
  imu_body_pose);

auto accelerations = imu.createAccelerationsFrame(ax, ay, az);
auto angular_speeds = imu.createAngularSpeedsFrame(wx, wy, wz);

romea::core::RollPitchKalmanEstimator estimator;

Eigen::Vector3d acceleration_vector(
  accelerations.accelerationAlongXAxis,
  accelerations.accelerationAlongYAxis,
  accelerations.accelerationAlongZAxis);

estimator.init(stamp, acceleration_vector, imu.getAccelerationVariance());

// ...
// Later, update the estimator with acceleration and angular-speed samples.
// ...
```

---

## Related packages

| Package | Role |
| ------- | ---- |
| `romea_core_common` | Shared core types used for time, monitoring and common utilities. |
| `romea_core_localisation_imu` | Builds localisation observations from IMU measurements. |

---

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

This library was developed by **Jean Laneurit** in the context of several research projects carried out at INRAE.
