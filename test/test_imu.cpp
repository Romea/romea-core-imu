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

// std
#include <cmath>

// gtest
#include "gtest/gtest.h"

// romea
#include "romea_core_imu/IMU6DOF.hpp"
#include "romea_core_imu/IMU9DOF.hpp"
#include "romea_core_imu/IMUAHRS.hpp"
#include "romea_core_imu/IMUVRU.hpp"

namespace
{

constexpr double EPSILON = 1e-12;

Eigen::Affine3d makeYawRotation(const double & angle)
{
  return Eigen::Affine3d(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
}

}  // namespace

TEST(TestIMU6DOF, testNoiseAndRangeAccessors)
{
  romea::core::IMU6DOF imu(100., 0.02, 0.1, 16., 0.01, 0.05, 8.);

  EXPECT_DOUBLE_EQ(imu.getRate(), 100.);
  EXPECT_DOUBLE_EQ(imu.getAccelerationNoiseDensity(), 0.02);
  EXPECT_DOUBLE_EQ(imu.getAccelerationBiasStabilityStd(), 0.1);
  EXPECT_DOUBLE_EQ(imu.getAccelerationRange(), 16.);
  EXPECT_DOUBLE_EQ(imu.getAngularSpeedNoiseDensity(), 0.01);
  EXPECT_DOUBLE_EQ(imu.getAngularSpeedBiasStabilityStd(), 0.05);
  EXPECT_DOUBLE_EQ(imu.getAngularSpeedRange(), 8.);

  const double expectedAccelerationStd = 1.57 * 0.02 * std::sqrt(100.) + 0.1;
  const double expectedAngularSpeedStd = 1.57 * 0.01 * std::sqrt(100.) + 0.05;

  EXPECT_DOUBLE_EQ(imu.getAccelerationStd(), expectedAccelerationStd);
  EXPECT_DOUBLE_EQ(imu.getAngularSpeedStd(), expectedAngularSpeedStd);
  EXPECT_DOUBLE_EQ(
    imu.getAccelerationVariance(), expectedAccelerationStd * expectedAccelerationStd);
  EXPECT_DOUBLE_EQ(
    imu.getAngularSpeedVariance(), expectedAngularSpeedStd * expectedAngularSpeedStd);
}

TEST(TestIMU6DOF, testBodyPoseRotatesAccelerationAndAngularSpeedFrames)
{
  romea::core::IMU6DOF imu(100., 0., 0., 16., 0., 0., 8., makeYawRotation(M_PI / 2.));

  const auto accelerations = imu.createAccelerationsFrame(1., 2., 3.);
  EXPECT_NEAR(accelerations.accelerationAlongXAxis, -2., EPSILON);
  EXPECT_NEAR(accelerations.accelerationAlongYAxis, 1., EPSILON);
  EXPECT_NEAR(accelerations.accelerationAlongZAxis, 3., EPSILON);

  const auto angularSpeeds = imu.createAngularSpeedsFrame(4., 5., 6.);
  EXPECT_NEAR(angularSpeeds.angularSpeedAroundXAxis, -5., EPSILON);
  EXPECT_NEAR(angularSpeeds.angularSpeedAroundYAxis, 4., EPSILON);
  EXPECT_NEAR(angularSpeeds.angularSpeedAroundZAxis, 6., EPSILON);

  imu.setBodyPose(Eigen::Affine3d::Identity());
  EXPECT_TRUE(imu.getBodyPose().matrix().isApprox(Eigen::Matrix4d::Identity()));
}

TEST(TestIMU9DOF, testMagneticsAreCompensatedRotatedAndCheckedAgainstRange)
{
  romea::core::IMU9DOF imu(
    100., 0., 0., 16., 0., 0., 8., 0.03, 0.2, 5., makeYawRotation(M_PI / 2.));

  Eigen::Affine3d softIronCompensation = Eigen::Affine3d::Identity();
  softIronCompensation.linear() = Eigen::Vector3d(2., 3., 4.).asDiagonal();
  imu.setSoftIronCompensationMatrix(softIronCompensation);

  const auto magnetics = imu.createMagneticsFrame(1., 2., 3.);
  EXPECT_NEAR(magnetics.magneticAlongXAxis, -6., EPSILON);
  EXPECT_NEAR(magnetics.magneticAlongYAxis, 2., EPSILON);
  EXPECT_NEAR(magnetics.magneticAlongZAxis, 12., EPSILON);

  EXPECT_TRUE(imu.isMagneticsOutOfRange(magnetics));
  EXPECT_FALSE(imu.isMagneticsOutOfRange({4.9, -5., 0.}));

  const double expectedMagneticStd = 1.57 * 0.03 * std::sqrt(100.) + 0.2;
  EXPECT_DOUBLE_EQ(imu.getMagneticNoiseDensity(), 0.03);
  EXPECT_DOUBLE_EQ(imu.getMagneticBiasStabilityStd(), 0.2);
  EXPECT_DOUBLE_EQ(imu.getMagneticRange(), 5.);
  EXPECT_DOUBLE_EQ(imu.getMagneticStd(), expectedMagneticStd);
  EXPECT_DOUBLE_EQ(imu.getMagneticVariance(), expectedMagneticStd * expectedMagneticStd);
  EXPECT_TRUE(imu.getSoftIronCompensationMatrix().matrix().isApprox(softIronCompensation.matrix()));
}

TEST(TestIMUVRUAndIMUAHRS, testAngleFramesAndVariances)
{
  romea::core::IMUVRU vru(100., 0., 0., 16., 0., 0., 8., 0.05);
  const auto rollPitchFrame = vru.createFrame(0.1, -0.2);

  EXPECT_NEAR(rollPitchFrame.rollAngle, 0.1, EPSILON);
  EXPECT_NEAR(rollPitchFrame.pitchAngle, -0.2, EPSILON);
  EXPECT_DOUBLE_EQ(vru.getAngleStd(), 0.05);
  EXPECT_DOUBLE_EQ(vru.getAngleVariance(), 0.05 * 0.05);

  romea::core::IMUAHRS ahrs(100., 0., 0., 16., 0., 0., 8., 0., 0., 5., 0.04);
  const auto rollPitchCourseFrame = ahrs.createFrame(0.1, -0.2, 0.3);

  EXPECT_NEAR(rollPitchCourseFrame.rollAngle, 0.1, EPSILON);
  EXPECT_NEAR(rollPitchCourseFrame.pitchAngle, -0.2, EPSILON);
  EXPECT_NEAR(rollPitchCourseFrame.courseAngle, 0.3, EPSILON);
  EXPECT_DOUBLE_EQ(ahrs.getAngleStd(), 0.04);
  EXPECT_DOUBLE_EQ(ahrs.getAngleVariance(), 0.04 * 0.04);
}
