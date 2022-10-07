#ifndef romea_IMU9DOF_hpp
#define romea_IMU9DOF_hpp

//romea
#include "IMU6DOF.hpp"
#include "MagneticsFrame.hpp"

//eigen
#include <Eigen/Eigen>

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
          const Eigen::Affine3d & bodyPose= Eigen::Affine3d::Identity());

  virtual ~IMU9DOF()=default;

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

}

#endif
