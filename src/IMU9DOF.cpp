//romea
#include "romea_imu/IMU9DOF.hpp"
#include <romea_common/signal/Noise.hpp>

namespace romea {

//--------------------------------------------------------------------
IMU9DOF::IMU9DOF(const double & rate,
                 const double & accelerationNoiseDensity,
                 const double & accelerationBiasStatibilityStd,
                 const double & accelerationRange,
                 const double & angularSpeedNoiseDensity,
                 const double & angularSpeedBiasStatibilityStd,
                 const double & angularSpeedRange,
                 const double & magneticNoiseDensity,
                 const double & magneticBiasStatibilityStd,
                 const double & magneticRange):
  IMU6DOF(rate,
          accelerationNoiseDensity,
          accelerationBiasStatibilityStd,
          accelerationRange,
          angularSpeedNoiseDensity,
          angularSpeedBiasStatibilityStd,
          angularSpeedRange),
  magneticNoiseDenity_(magneticNoiseDensity),
  magneticBiasStatibilityStd_(magneticBiasStatibilityStd),
  magneticRange_(magneticRange),
  softIronCompensationMatrix_(Eigen::Affine3d::Identity())
{

}

//--------------------------------------------------------------------
MagneticsFrame IMU9DOF::createMagneticsFrame(const double &magneticAlongXAxis,
                                             const double &magneticAlongYAxis,
                                             const double &magneticAlongZAxis)
{

  Eigen::Vector3d magnetics(magneticAlongXAxis,
                            magneticAlongYAxis,
                            magneticAlongZAxis);

  magnetics = rigidTransformation_.rotation()*softIronCompensationMatrix_*magnetics;

  return {magnetics.x(),magnetics.y(),magnetics.z()};

}

//--------------------------------------------------------------------
bool IMU9DOF::isMagneticsOutOfRange(const MagneticsFrame & magneticsFrame) const
{
  return std::abs(magneticsFrame.magneticAlongXAxis)>magneticRange_||
      std::abs(magneticsFrame.magneticAlongYAxis)>magneticRange_ ||
      std::abs(magneticsFrame.magneticAlongZAxis)>magneticRange_;
}

//--------------------------------------------------------------------
double IMU9DOF::getMagneticStd() const
{
  assert(rate_>0);
  return convertNoiseDensityToNoiseStd(magneticNoiseDenity_,rate_)+
      magneticBiasStatibilityStd_;
}

//--------------------------------------------------------------------
double IMU9DOF::getMagneticVariance() const
{
  double magneticStd = getMagneticStd();
  return magneticStd*magneticStd;
}

//--------------------------------------------------------------------
const double & IMU9DOF::getMagneticNoiseDenity()const
{
  return magneticNoiseDenity_;
}

//--------------------------------------------------------------------
const double & IMU9DOF::getMagneticBiasStatibilityStd()const
{
  return magneticBiasStatibilityStd_;
}

//--------------------------------------------------------------------
const double & IMU9DOF::getMagneticRange()const
{
  return magneticRange_;
}

//--------------------------------------------------------------------
void IMU9DOF::setSoftIronCompensationMatrix(const Eigen::Affine3d &matrix)
{
  softIronCompensationMatrix_=matrix;
}

//--------------------------------------------------------------------
const Eigen::Affine3d & IMU9DOF::getSoftIronCompensationMatrix() const
{
  return softIronCompensationMatrix_;
}

}
