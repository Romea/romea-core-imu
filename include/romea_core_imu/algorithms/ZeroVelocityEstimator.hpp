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


#ifndef ROMEA_CORE_IMU__ALGORITHMS__ZEROVELOCITYESTIMATOR_HPP_
#define ROMEA_CORE_IMU__ALGORITHMS__ZEROVELOCITYESTIMATOR_HPP_

// Eigen
#include <Eigen/Eigen>

// romea
#include <romea_core_common/monitoring/OnlineVariance.hpp>

namespace romea
{

class ZeroVelocityEstimator
{
public:
  ZeroVelocityEstimator(const double &imuRate,
                        const double & accelerationSpeedStd,
                        const double & angularSpeedStd);

  void init(const double & accelerationSpeedStd,
            const double & angularSpeedStd);


  bool update(const double & accelerationAlongXBodyAxis,
              const double & accelerationAlongYBodyAxis,
              const double & accelerationAlongZBodyAxis,
              const double & angularSpeedAroundXBodyAxis,
              const double & angularSpeedAroundYBodyAxis,
              const double & angularSpeedAroundZBodyAxis);

  bool update(const Eigen::Vector3d & accelerationSpeeds,
              const Eigen::Vector3d & angularSpeeds);

  double getAccelerationStd()const;

  double getAngularSpeedStd()const;

  void reset();

private:
  double accelerationVarianceThreshold_;
  double angularSpeedVarianceThreshold_;

  OnlineVariance varAccelerationAlongXBodyAxis_;
  OnlineVariance varAccelerationAlongYBodyAxis_;
  OnlineVariance varAccelerationAlongZBodyAxis_;
  OnlineVariance varAngularSpeedAroundXBodyAxis_;
  OnlineVariance varAngularSpeedAroundYBodyAxis_;
  OnlineVariance varAngularSpeedAroundZBodyAxis_;
};

}  // namespace romea

#endif  // ROMEA_CORE_IMU__ALGORITHMS__ZEROVELOCITYESTIMATOR_HPP_
