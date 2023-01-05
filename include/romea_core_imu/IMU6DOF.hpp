// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_IMU__IMU6DOF_HPP_
#define ROMEA_CORE_IMU__IMU6DOF_HPP_

// eigen
#include <Eigen/Eigen>
#include <Eigen/Geometry>

// romea
#include "romea_core_imu/AccelerationsFrame.hpp"
#include "romea_core_imu/AngularSpeedsFrame.hpp"

namespace romea
{

class IMU6DOF
{
public:
  IMU6DOF(
    const double & rate,
    const double & accelerationNoiseDensity,
    const double & accelerationBiasStatibilityStd,
    const double & accelerationRange,
    const double & angularSpeedNoiseDensity,
    const double & angularSpeedBiasStatibilityStd,
    const double & angularSpeedRange,
    const Eigen::Affine3d & bodyPose = Eigen::Affine3d::Identity());

  virtual ~IMU6DOF() = default;

public:
  AccelerationsFrame createAccelerationsFrame(
    const double & accelerationAlongXAxis,
    const double & accelerationAlongYAxis,
    const double & accelerationAlongZAxis);

  AngularSpeedsFrame createAngularSpeedsFrame(
    const double & angularSpeedAroundXAxis,
    const double & angularSpeedAroundYAxis,
    const double & angularSpeedAroundZAxis);

public:
  const double & getRate() const;

  double getAccelerationStd() const;
  double getAccelerationVariance() const;
  const double & getAccelerationRange() const;
  const double & getAccelerationNoiseDensity()const;
  const double & getAccelerationBiasStatibilityStd()const;

  double getAngularSpeedStd() const;
  double getAngularSpeedVariance() const;
  const double & getAngularSpeedRange() const;
  const double & getAngularSpeedNoiseDensity()const;
  const double & getAngularSpeedBiasStatibilityStd()const;

  void setBodyPose(const Eigen::Affine3d & rigidTransformation);
  const Eigen::Affine3d & getBodyPose()const;

protected:
  double rate_;

  double accelerationNoiseDensity_;
  double accelerationBiasStatibilityStd_;
  double accelerationRange_;

  double angularSpeedNoiseDensity_;
  double angularSpeedBiasStatibilityStd_;
  double angularSpeedRange_;

  Eigen::Affine3d rigidTransformation_;
};

}  // namespace romea

#endif   // ROMEA_CORE_IMU_IMU6DOF_HPP_
