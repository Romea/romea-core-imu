// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_IMU__ANGULARSPEEDSFRAME_HPP_
#define ROMEA_CORE_IMU__ANGULARSPEEDSFRAME_HPP_

// std
#include <memory>

namespace romea {

struct AngularSpeedsFrame
{
  using Ptr = std::shared_ptr<AngularSpeedsFrame> ;

  double angularSpeedAroundXAxis;
  double angularSpeedAroundYAxis;
  double angularSpeedAroundZAxis;
};

}  // namespace romea

#endif  // ROMEA_CORE_IMU__ANGULARSPEEDSFRAME_HPP_
