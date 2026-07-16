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
#include "romea_core_imu/algorithms/ZeroVelocityEstimator.hpp"

namespace
{

constexpr double EPSILON = 1e-12;

}  // namespace

TEST(TestZeroVelocityEstimator, testRequiresFullWindowBeforeReportingZeroVelocity)
{
  romea::core::ZeroVelocityEstimator estimator(5., 0.1, 0.01);

  for (size_t n = 0; n < 9; ++n) {
    EXPECT_FALSE(estimator.update(Eigen::Vector3d(0., 0., -9.81), Eigen::Vector3d::Zero()));
  }

  EXPECT_TRUE(estimator.update(Eigen::Vector3d(0., 0., -9.81), Eigen::Vector3d::Zero()));
  EXPECT_DOUBLE_EQ(estimator.getAccelerationStd(), 0.);
  EXPECT_DOUBLE_EQ(estimator.getAngularSpeedStd(), 0.);
}

TEST(TestZeroVelocityEstimator, testDetectsMotionAndCanBeReset)
{
  romea::core::ZeroVelocityEstimator estimator(5., 0.1, 0.01);

  for (size_t n = 0; n < 10; ++n) {
    const double sign = n % 2 == 0 ? 1. : -1.;
    estimator.update(Eigen::Vector3d(sign, -sign, -9.81 + sign), Eigen::Vector3d(sign, 0., -sign));
  }

  EXPECT_FALSE(estimator.update(Eigen::Vector3d(1., -1., -8.81), Eigen::Vector3d(1., 0., -1.)));

  estimator.reset();
  EXPECT_FALSE(estimator.update(Eigen::Vector3d(0., 0., -9.81), Eigen::Vector3d::Zero()));
}

TEST(TestZeroVelocityEstimator, testStandardDeviationsUseAllThreeAxes)
{
  romea::core::ZeroVelocityEstimator estimator(2., 10., 10.);

  estimator.update(Eigen::Vector3d(0., 0., 0.), Eigen::Vector3d(0., 0., 0.));
  estimator.update(Eigen::Vector3d(0., 1., 2.), Eigen::Vector3d(1., 2., 3.));
  estimator.update(Eigen::Vector3d(0., 0., 0.), Eigen::Vector3d(0., 0., 0.));
  estimator.update(Eigen::Vector3d(0., 1., 2.), Eigen::Vector3d(1., 2., 3.));

  EXPECT_NEAR(estimator.getAccelerationStd(), std::sqrt(5. / 3.), EPSILON);
  EXPECT_NEAR(estimator.getAngularSpeedStd(), std::sqrt(14. / 3.), EPSILON);
}
