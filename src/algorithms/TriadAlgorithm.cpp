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


// romea
#include "romea_core_imu/algorithms/TriadAlgorithm.hpp"

namespace romea
{
namespace core
{

//--------------------------------------------------------------------
TriadAttitude::TriadAttitude()
: currentAttitude_(Eigen::Matrix3d::Identity()),
  referenceAttitude_(Eigen::Matrix3d::Identity()),
  isInitialized_(false)
{
}

//--------------------------------------------------------------------
TriadAttitude::TriadAttitude(
  const Eigen::Vector3d & imuAccelerations,
  const Eigen::Vector3d & imuMagnetics)
: currentAttitude_(Eigen::Matrix3d::Identity()),
  referenceAttitude_(Eigen::Matrix3d::Identity()),
  isInitialized_(true)
{
  init(imuAccelerations, imuMagnetics);
}

//--------------------------------------------------------------------
inline void computeAttitude(
  const Eigen::Vector3d & imuAccelerations,
  const Eigen::Vector3d & imuMagnetics,
  Eigen::Matrix3d & R)
{
  R.col(2) = imuAccelerations.normalized();
  R.col(1) = (imuAccelerations.cross(imuMagnetics)).normalized();
  R.col(0) = (R.col(1)).cross(R.col(2));
}

//--------------------------------------------------------------------
void TriadAttitude::init(
  const Eigen::Vector3d & imuAccelerations,
  const Eigen::Vector3d & imuMagnetics)
{
  computeAttitude(imuAccelerations, imuMagnetics, referenceAttitude_);
  isInitialized_ = true;
}


//--------------------------------------------------------------------
Eigen::Matrix3d TriadAttitude::compute(
  const Eigen::Vector3d & imuAccelerations,
  const Eigen::Vector3d & imuMagnetics)
{
  assert(isInitialized_);
  computeAttitude(imuAccelerations, imuMagnetics, currentAttitude_);
  return currentAttitude_ * referenceAttitude_.inverse();
}

//--------------------------------------------------------------------
bool TriadAttitude::isInitialized()const
{
  return isInitialized_;
}

}  // namespace core
}  // namespace romea
