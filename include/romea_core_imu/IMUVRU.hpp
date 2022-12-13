#ifndef ROMEA_CORE_IMU_IMUVRU_HPP_
#define ROMEA_CORE_IMU_IMUVRU_HPP_

// eigen
#include <Eigen/Eigen>

// romea
#include "romea_core_imu/IMU6DOF.hpp"
#include "romea_core_imu/RollPitchFrame.hpp"

namespace romea {

class IMUVRU : public IMU6DOF
{
public :

  IMUVRU(const double & rate,
         const double & accelerationNoiseDensity,
         const double & accelerationBiasStatibilityStd,
         const double & accelerationRange,
         const double & angularSpeedNoiseDensity,
         const double & angularSpeedBiasStatibilityStd,
         const double & angularSpeedRange,
         const double & angleStd,
         const Eigen::Affine3d & bodyPose = Eigen::Affine3d::Identity());

public:
  RollPitchFrame createFrame(const double & rollAngle,
                             const double & pitchAngle);


  double getAngleStd() const;
  double getAngleVariance() const;

private:
  const double angleStd_;
  const double angleVariance_;
};

}  // namespace romea

#endif  // ROMEA_CORE_IMU_IMUVRU_HPP_
