// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_IMU__IMUAHRS_HPP_
#define ROMEA_CORE_IMU__IMUAHRS_HPP_

// eigen
#include <Eigen/Eigen>

// romea
#include "romea_core_imu/IMU9DOF.hpp"
#include "romea_core_imu/RollPitchCourseFrame.hpp"


namespace romea
{

class IMUAHRS : public IMU9DOF
{
public:
  IMUAHRS(
    const double & rate,
    const double & accelerationNoiseDensity,
    const double & accelerationBiaisStatibilityStd,
    const double & accelerationRange,
    const double & angularSpeedNoiseDensity,
    const double & angularSpeedBiaisStatibilityStd,
    const double & angularSpeedRange,
    const double & magneticNoiseDenity,
    const double & magneticBiaisStatibilityStd,
    const double & magneticRange,
    const double & angleStd,
    const Eigen::Affine3d & bodyPose = Eigen::Affine3d::Identity());

public:
  RollPitchCourseFrame createFrame(
    const double & rollAngle,
    const double & pitchAngle,
    const double & courseAngle);

  double getAngleStd() const;
  double getAngleVariance() const;

private:
  const double angleStd_;
  const double angleVariance_;
};

}  // namespace romea

#endif  // ROMEA_CORE_IMU_IMUAHRS_HPP_
