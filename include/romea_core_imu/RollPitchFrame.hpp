// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_IMU__ROLLPITCHFRAME_HPP
#define ROMEA_CORE_IMU__ROLLPITCHFRAME_HPP

// eigen
#include <Eigen/Eigen>

// std
#include <memory>

namespace romea
{

struct RollPitchFrame
{
  using Ptr = std::shared_ptr<RollPitchFrame>;
  double rollAngle;
  double pitchAngle;
};

}  // namespace romea

#endif  // ROMEA_CORE_IMU__ROLLPITCHFRAME_HPP
