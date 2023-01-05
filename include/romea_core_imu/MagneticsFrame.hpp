// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CORE_IMU__MAGNETICSFRAME_HPP_
#define ROMEA_CORE_IMU__MAGNETICSFRAME_HPP_

// std
#include <memory>

namespace romea {

struct MagneticsFrame
{
  using Ptr = std::shared_ptr<MagneticsFrame> ;

  double magneticAlongXAxis;
  double magneticAlongYAxis;
  double magneticAlongZAxis;
};

}  // namespace romea

#endif  // ROMEA_CORE_IMU__MAGNETICSFRAME_HPP_
