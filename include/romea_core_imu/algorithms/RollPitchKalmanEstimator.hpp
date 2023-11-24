// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef ROMEA_CORE_IMU__ALGORITHMS__ROLLPITCHKALMANESTIMATOR_HPP_
#define ROMEA_CORE_IMU__ALGORITHMS__ROLLPITCHKALMANESTIMATOR_HPP_

// Eigen
#include <Eigen/Core>

// romea
#include "romea_core_common/time/Time.hpp"

namespace romea
{
namespace core
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

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_IMU__ALGORITHMS__ROLLPITCHKALMANESTIMATOR_HPP_
