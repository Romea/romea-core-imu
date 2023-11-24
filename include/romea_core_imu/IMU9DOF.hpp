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


#ifndef ROMEA_CORE_IMU__IMU9DOF_HPP_
#define ROMEA_CORE_IMU__IMU9DOF_HPP_

// eigen
#include <Eigen/Eigen>

// romea
#include "romea_core_imu/IMU6DOF.hpp"
#include "romea_core_imu/MagneticsFrame.hpp"


namespace romea
{
namespace core
{

class IMU9DOF : public IMU6DOF
{
public:
  IMU9DOF(
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
    const Eigen::Affine3d & bodyPose = Eigen::Affine3d::Identity());

  virtual ~IMU9DOF() = default;

public:
  MagneticsFrame createMagneticsFrame(
    const double & magneticAlongXAxis,
    const double & magneticAlongYAxis,
    const double & magneticAlongZAxis);

  bool isMagneticsOutOfRange(const MagneticsFrame & magneticsFrame) const;

public:
  double getMagneticStd() const;
  double getMagneticVariance() const;
  const double & getMagneticRange() const;
  const double & getMagneticNoiseDenity() const;
  const double & getMagneticBiasStatibilityStd() const;

  void setSoftIronCompensationMatrix(const Eigen::Affine3d & matrix);
  const Eigen::Affine3d & getSoftIronCompensationMatrix() const;

private:
  double magneticNoiseDenity_;
  double magneticBiasStatibilityStd_;
  double magneticRange_;

  Eigen::Affine3d softIronCompensationMatrix_;
};

}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_IMU__IMU9DOF_HPP_
