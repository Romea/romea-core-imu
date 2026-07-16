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

#include "romea_core_imu/IMUAHRS.hpp"

#include <romea_core_common/math/EulerAngles.hpp>

namespace romea
{
namespace core
{

//--------------------------------------------------------------------
IMUAHRS::IMUAHRS(
  const double & rate,
  const double & accelerationNoiseDensity,
  const double & accelerationBiasStabilityStd,
  const double & accelerationRange,
  const double & angularSpeedNoiseDensity,
  const double & angularSpeedBiasStabilityStd,
  const double & angularSpeedRange,
  const double & magneticNoiseDensity,
  const double & magneticBiasStabilityStd,
  const double & magneticRange,
  const double & angleStd,
  const Eigen::Affine3d & bodyPose)
: IMU9DOF(
    rate,
    accelerationNoiseDensity,
    accelerationBiasStabilityStd,
    accelerationRange,
    angularSpeedNoiseDensity,
    angularSpeedBiasStabilityStd,
    angularSpeedRange,
    magneticNoiseDensity,
    magneticBiasStabilityStd,
    magneticRange,
    bodyPose),
  angleStd_(angleStd),
  angleVariance_(angleStd * angleStd)
{
}

//--------------------------------------------------------------------
RollPitchCourseFrame IMUAHRS::createFrame(
  const double & rollAngle, const double & pitchAngle, const double & courseAngle)
{
  Eigen::Vector3d eulerAngles(rollAngle, pitchAngle, courseAngle);
  eulerAngles = rotation3DToEulerAngles(eulerAnglesToRotation3D(eulerAngles));

  // C++11
  RollPitchCourseFrame frame;
  frame.rollAngle = betweenMinusPiAndPi(eulerAngles.x());
  frame.pitchAngle = betweenMinusPiAndPi(eulerAngles.y());
  frame.courseAngle = eulerAngles.z();
  return frame;

  // C++17
  // return {{eulerAngles.x(),eulerAngles.y()},eulerAngles.z()};
}

//--------------------------------------------------------------------

double IMUAHRS::getAngleStd() const
{
  return angleStd_;
}

//--------------------------------------------------------------------
double IMUAHRS::getAngleVariance() const
{
  return angleVariance_;
}

}  // namespace core
}  // namespace romea
