// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_IMU__ALGORITHMS__ROLLPITCHKALMANESTIMATOR_HPP_
#define ROMEA_CORE_IMU__ALGORITHMS__ROLLPITCHKALMANESTIMATOR_HPP_

// Eigen
#include <Eigen/Core>

// romea
#include "romea_core_common/time/Time.hpp"

namespace romea
{

class RollPitchKalmanEstimator
{
public:
  RollPitchKalmanEstimator();

  void init(
    const Duration & duration,
    const Eigen::Vector3d & imuAccelerations,
    const double & imuAccelerationsVar);

  void update(
    const Duration & duration,
    const Eigen::Vector3d & imuAccelerations,
    const double & imuAccelerationsVar,
    const Eigen::Vector3d & imuAngularSpeeds,
    const double & imuAngularSpeedsVar);

  static double computeRoll(const Eigen::Vector3d & imuAccelerations);
  static double computePitch(const Eigen::Vector3d & imuAccelerations);

  double getRoll()const;
  double getPitch()const;
  double getRollVariance()const;
  double getPitchVariance()const;
  Eigen::Matrix2d getRollPitchCovariance()const;

  bool isInitialized();

private:
  Eigen::Vector2d X_;
  Eigen::Matrix2d P_;
  Eigen::MatrixXd F_;
  Eigen::MatrixXd H_;
  Eigen::Vector3d Inn_;
  Eigen::Matrix3d InnCov_;
  Eigen::MatrixXd K_;
  bool isInitialized_;
  Duration previousDuration_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace romea

#endif  // ROMEA_CORE_IMU__ALGORITHMS__ROLLPITCHKALMANESTIMATOR_HPP_
