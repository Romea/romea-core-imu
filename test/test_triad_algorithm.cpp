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
#include <stdexcept>

// gtest
#include "gtest/gtest.h"

// romea
#include "romea_core_imu/algorithms/TriadAlgorithm.hpp"

namespace
{

constexpr double EPSILON = 1e-12;

}  // namespace

TEST(TestTriadAttitude, testInitializationAndIdentityAttitude)
{
  romea::core::TriadAttitude triad;
  EXPECT_FALSE(triad.isInitialized());

  triad.init(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX());
  EXPECT_TRUE(triad.isInitialized());

  const Eigen::Matrix3d attitude =
    triad.compute(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX());
  EXPECT_TRUE(attitude.isApprox(Eigen::Matrix3d::Identity(), EPSILON));
}

TEST(TestTriadAttitude, testComputesRotationFromReferenceVectors)
{
  const Eigen::Vector3d acceleration = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d magnetic = Eigen::Vector3d::UnitX();
  const Eigen::Matrix3d expectedRotation =
    Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitZ()).toRotationMatrix();

  romea::core::TriadAttitude triad(acceleration, magnetic);
  const Eigen::Matrix3d attitude =
    triad.compute(expectedRotation * acceleration, expectedRotation * magnetic);

  EXPECT_TRUE(attitude.isApprox(expectedRotation, EPSILON));
  EXPECT_TRUE((attitude * attitude.transpose()).isApprox(Eigen::Matrix3d::Identity(), EPSILON));
  EXPECT_NEAR(attitude.determinant(), 1., EPSILON);
}

TEST(TestTriadAttitude, testRejectsDegeneratedInputs)
{
  romea::core::TriadAttitude triad;

  EXPECT_THROW(
    triad.init(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX()), std::invalid_argument);
  EXPECT_THROW(
    triad.init(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero()), std::invalid_argument);
  EXPECT_THROW(
    triad.init(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ()), std::invalid_argument);
}
