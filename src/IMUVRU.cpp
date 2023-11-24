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


#include "romea_core_imu/IMUVRU.hpp"
#include <romea_core_common/math/EulerAngles.hpp>

namespace romea
{
namespace core
{

//--------------------------------------------------------------------
IMUVRU::IMUVRU(
  const double & rate,
  const double & accelerationNoiseDensity,
  const double & accelerationBiasStatibilityStd,
  const double & accelerationRange,
  const double & angularSpeedNoiseDensity,
  const double & angularSpeedBiasStatibilityStd,
  const double & angularSpeedRange,
  const double & angleStd,
  const Eigen::Affine3d & bodyPose)
: IMU6DOF(rate,
    accelerationNoiseDensity,
    accelerationBiasStatibilityStd,
    accelerationRange,
    angularSpeedNoiseDensity,
    angularSpeedBiasStatibilityStd,
    angularSpeedRange,
    bodyPose),
  angleStd_(angleStd),
  angleVariance_(angleStd * angleStd)
{
}

//--------------------------------------------------------------------
RollPitchFrame IMUVRU::createFrame(
  const double & rollAngle,
  const double & pitchAngle)
{
  Eigen::Vector3d eulerAngles(rollAngle, pitchAngle, 0);
  eulerAngles = rotation3DToEulerAngles(eulerAnglesToRotation3D(eulerAngles));
  return {eulerAngles.x(), eulerAngles.y()};
}

//--------------------------------------------------------------------
double IMUVRU::getAngleStd() const
{
  return angleStd_;
}

//--------------------------------------------------------------------
double IMUVRU::getAngleVariance() const
{
  return angleVariance_;
}

}  // namespace core
}  // namespace romea
