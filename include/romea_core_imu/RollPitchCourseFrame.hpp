#ifndef ROMEA_CORE_IMU__ROLLPITCHCOURSEFRAME_HPP_
#define ROMEA_CORE_IMU__ROLLPITCHCOURSEFRAME_HPP_

// std
#include <memory>

// romea
#include "romea_core_imu/RollPitchFrame.hpp"

namespace romea
{

struct RollPitchCourseFrame : RollPitchFrame
{
  using Ptr = std::shared_ptr<RollPitchCourseFrame>;
  double courseAngle;
};

}  // namespace romea

#endif  // ROMEA_CORE_IMU__ROLLPITCHCOURSEFRAME_HPP_
