#ifndef ROMEA_CORE_IMU_ROLLPITCHFRAME_HPP
#define ROMEA_CORE_IMU_ROLLPITCHFRAME_HPP

// eigen
#include <Eigen/Eigen>

// std
#include <memory>

namespace romea {


struct RollPitchFrame
{
  using Ptr = std::shared_ptr<RollPitchFrame> ;
  double rollAngle;
  double pitchAngle;
};

}  // namespace romea

#endif  // ROMEA_CORE_IMU_ROLLPITCHFRAME_HPP
