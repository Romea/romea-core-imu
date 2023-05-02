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


#ifndef ROMEA_CORE_IMU__IMUVRU_HPP_
#define ROMEA_CORE_IMU__IMUVRU_HPP_

// eigen
#include <Eigen/Eigen>

// romea
#include "romea_core_imu/IMU6DOF.hpp"
#include "romea_core_imu/RollPitchFrame.hpp"

namespace romea
{

class IMUVRU : public IMU6DOF
{
public:
  IMUVRU(
    const double & rate,
    const double & accelerationNoiseDensity,
    const double & accelerationBiasStatibilityStd,
    const double & accelerationRange,
    const double & angularSpeedNoiseDensity,
    const double & angularSpeedBiasStatibilityStd,
    const double & angularSpeedRange,
    const double & angleStd,
    const Eigen::Affine3d & bodyPose = Eigen::Affine3d::Identity());

public:
  RollPitchFrame createFrame(
    const double & rollAngle,
    const double & pitchAngle);


  double getAngleStd() const;
  double getAngleVariance() const;

private:
  const double angleStd_;
  const double angleVariance_;
};

}  // namespace romea

#endif  // ROMEA_CORE_IMU__IMUVRU_HPP_
