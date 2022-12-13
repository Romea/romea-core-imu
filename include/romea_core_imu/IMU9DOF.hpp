#ifndef ROMEA_CORE_IMU_IMU9DOF_HPP_
#define ROMEA_CORE_IMU_IMU9DOF_HPP_

// eigen
#include <Eigen/Eigen>

// romea
#include "romea_core_imu/IMU6DOF.hpp"
#include "romea_core_imu/MagneticsFrame.hpp"


namespace romea {

class IMU9DOF : public IMU6DOF
{
public :
  IMU9DOF(const double & rate,
          const double & accelerationNoiseDensity,
          const double & accelerationBiaisStatibilityStd,
          const double & accelerationRange,
          const double & angularSpeedNoiseDensity,
          const double & angularSpeedBiaisStatibilityStd,
          const double & angularSpeedRange,
          const double & magneticNoiseDenity,
          const double & magneticBiaisStatibilityStd,
          const double & magneticRange,
          const Eigen::Affine3d & bodyPose = Eigen::Affine3d::Identity());

  virtual ~IMU9DOF() = default;

public:
  MagneticsFrame createMagneticsFrame(const double & magneticAlongXAxis,
                                      const double & magneticAlongYAxis,
                                      const double & magneticAlongZAxis);

  bool isMagneticsOutOfRange(const MagneticsFrame & magneticsFrame) const;

public :

  double getMagneticStd() const;
  double getMagneticVariance() const;
  const double & getMagneticRange() const;
  const double & getMagneticNoiseDenity() const;
  const double & getMagneticBiasStatibilityStd() const;

  void setSoftIronCompensationMatrix(const Eigen::Affine3d &matrix);
  const Eigen::Affine3d & getSoftIronCompensationMatrix() const;

private:
  double magneticNoiseDenity_;
  double magneticBiasStatibilityStd_;
  double magneticRange_;

  Eigen::Affine3d softIronCompensationMatrix_;
};

}  // namespace romea

#endif  // ROMEA_CORE_IMU_IMU9DOF_HPP_
