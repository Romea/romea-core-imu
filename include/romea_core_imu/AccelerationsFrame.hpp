#ifndef ROMEA_CORE_IMU_ACCELERATIONSFRAME_HPP_
#define ROMEA_CORE_IMU_ACCELERATIONSFRAME_HPP_

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

#endif  // ROMEA_CORE_IMU_ACCELERATIONSFRAME_HPP_
