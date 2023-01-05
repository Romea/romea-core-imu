// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_IMU__ACCELERATIONSFRAME_HPP_
#define ROMEA_CORE_IMU__ACCELERATIONSFRAME_HPP_

// std
#include <memory>

namespace romea {

struct AccelerationsFrame
{
  using Ptr = std::shared_ptr<AccelerationsFrame> ;

  double accelerationAlongXAxis;
  double accelerationAlongYAxis;
  double accelerationAlongZAxis;
};

}  // namespace romea

#endif  // ROMEA_CORE_IMU__ACCELERATIONSFRAME_HPP_
