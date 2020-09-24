//romea
#include "romea_imu/IMU6DOF.hpp"
#include <romea_common/signal/Noise.hpp>

namespace romea {


//--------------------------------------------------------------------
IMU6DOF::IMU6DOF(const double & rate,
                 const double & accelerationNoiseDensity,
                 const double & accelerationBiasStatibilityStd,
                 const double & accelerationRange,
                 const double & angularSpeedNoiseDensity,
                 const double & angularSpeedBiasStatibilityStd,
                 const double & angularSpeedRange):
  rate_(rate),
  accelerationNoiseDensity_(accelerationNoiseDensity),
  accelerationBiasStatibilityStd_(accelerationBiasStatibilityStd),
  accelerationRange_(accelerationRange),
  angularSpeedNoiseDensity_(angularSpeedNoiseDensity),
  angularSpeedBiasStatibilityStd_(angularSpeedBiasStatibilityStd),
  angularSpeedRange_(angularSpeedRange),
  rigidTransformation_(Eigen::Affine3d::Identity())
{


}


//--------------------------------------------------------------------
AccelerationsFrame IMU6DOF::createAccelerationsFrame(const double & accelerationAlongXAxis,
                                                     const double & accelerationAlongYAxis,
                                                     const double & accelerationAlongZAxis)
{
  Eigen::Vector3d accelerations(accelerationAlongXAxis,
                                accelerationAlongYAxis,
                                accelerationAlongZAxis);

  accelerations= rigidTransformation_.rotation()*accelerations;

  return {accelerations.x(),accelerations.y(),accelerations.z()};
}

//--------------------------------------------------------------------
AngularSpeedsFrame IMU6DOF::createAngularSpeedsFrame(const double & angularSpeedAroundXAxis,
                                                     const double & angularSpeedAroundYAxis,
                                                     const double & angularSpeedAroundZAxis)
{
  Eigen::Vector3d angularSpeeds(angularSpeedAroundXAxis,
                                angularSpeedAroundYAxis,
                                angularSpeedAroundZAxis);

  angularSpeeds = rigidTransformation_.rotation()*angularSpeeds;

  return {angularSpeeds.x(),angularSpeeds.y(),angularSpeeds.z()};
}


//--------------------------------------------------------------------
const double & IMU6DOF::getRate() const
{
  return rate_;
}

//--------------------------------------------------------------------

double IMU6DOF::getAccelerationStd() const
{
  assert(rate_>0);
  return convertNoiseDensityToNoiseStd(accelerationNoiseDensity_,rate_)+
      accelerationBiasStatibilityStd_;
}

//--------------------------------------------------------------------
double IMU6DOF::getAccelerationVariance() const
{
  double accelerationStd = getAccelerationStd();
  return accelerationStd*accelerationStd;
}

//--------------------------------------------------------------------
const double & IMU6DOF::getAccelerationRange() const
{
  return accelerationRange_;
}

//--------------------------------------------------------------------
double IMU6DOF::getAngularSpeedStd() const
{
  assert(rate_>0);
  return convertNoiseDensityToNoiseStd(angularSpeedNoiseDensity_,rate_)+
      angularSpeedBiasStatibilityStd_;
}

//--------------------------------------------------------------------
double IMU6DOF::getAngularSpeedVariance() const
{
  double angularSpeedStd_ = getAngularSpeedStd();
  return angularSpeedStd_*angularSpeedStd_;
}

//--------------------------------------------------------------------
const double & IMU6DOF::getAngularSpeedRange() const
{
  return angularSpeedRange_;
}


//--------------------------------------------------------------------
const double & IMU6DOF::getAccelerationNoiseDensity()const
{
  return accelerationNoiseDensity_;
}

//--------------------------------------------------------------------
const double & IMU6DOF::getAccelerationBiasStatibilityStd() const
{
  return accelerationBiasStatibilityStd_;
}

//--------------------------------------------------------------------
const double & IMU6DOF::getAngularSpeedNoiseDensity() const
{
  return angularSpeedNoiseDensity_;
}

//--------------------------------------------------------------------
const double & IMU6DOF::getAngularSpeedBiasStatibilityStd() const
{
  return angularSpeedBiasStatibilityStd_;
}

//--------------------------------------------------------------------
void IMU6DOF::setBodyPose(const Eigen::Affine3d & rigidTransformation)
{
  rigidTransformation_=rigidTransformation;
}

//--------------------------------------------------------------------
const Eigen::Affine3d & IMU6DOF::getBodyPose()const
{
  return rigidTransformation_;
}

}
