#include "romea_core_imu/algorithms/ZeroVelocityEstimator.hpp"

#include <cmath>

namespace {
const double AVERAGE_PRECISION = 0.001;
}

namespace romea
{

//-----------------------------------------------------------------------------
ZeroVelocityEstimator::ZeroVelocityEstimator(const double & imuRate,
                                             const double & accelerationSpeedStd,
                                             const double & angularSpeedStd):
  accelerationVarianceThreshold_(2*accelerationSpeedStd*accelerationSpeedStd),
  angularSpeedVarianceThreshold_(2*angularSpeedStd*angularSpeedStd),
  varAccelerationAlongXBodyAxis_(AVERAGE_PRECISION,2*imuRate),
  varAccelerationAlongYBodyAxis_(AVERAGE_PRECISION,2*imuRate),
  varAccelerationAlongZBodyAxis_(AVERAGE_PRECISION,2*imuRate),
  varAngularSpeedAroundXBodyAxis_(AVERAGE_PRECISION,2*imuRate),
  varAngularSpeedAroundYBodyAxis_(AVERAGE_PRECISION,2*imuRate),
  varAngularSpeedAroundZBodyAxis_(AVERAGE_PRECISION,2*imuRate)
{

}

//-----------------------------------------------------------------------------
void ZeroVelocityEstimator::init(const double & accelerationSpeedStd,
                                 const double & angularSpeedStd)
{
  accelerationVarianceThreshold_= 2*accelerationSpeedStd*accelerationSpeedStd;
  angularSpeedVarianceThreshold_ = 2*angularSpeedStd*angularSpeedStd;
}


//-----------------------------------------------------------------------------
bool ZeroVelocityEstimator::update(const double & accelerationAlongXBodyAxis,
                                   const double & accelerationAlongYBodyAxis,
                                   const double & accelerationAlongZBodyAxis,
                                   const double & angularSpeedAroundXBodyAxis,
                                   const double & angularSpeedAroundYBodyAxis,
                                   const double & angularSpeedAroundZBodyAxis)
{
  varAccelerationAlongXBodyAxis_.update(accelerationAlongXBodyAxis);
  varAccelerationAlongYBodyAxis_.update(accelerationAlongYBodyAxis);
  varAccelerationAlongZBodyAxis_.update(accelerationAlongZBodyAxis);
  varAngularSpeedAroundXBodyAxis_.update(angularSpeedAroundXBodyAxis);
  varAngularSpeedAroundYBodyAxis_.update(angularSpeedAroundYBodyAxis);
  varAngularSpeedAroundZBodyAxis_.update(angularSpeedAroundZBodyAxis);

  if(varAccelerationAlongXBodyAxis_.isAvailable())
  {
    return varAccelerationAlongXBodyAxis_.getVariance()<accelerationVarianceThreshold_ &&
        varAccelerationAlongYBodyAxis_.getVariance()<accelerationVarianceThreshold_ &&
        varAccelerationAlongZBodyAxis_.getVariance()<accelerationVarianceThreshold_ &&
        varAngularSpeedAroundXBodyAxis_.getVariance()< angularSpeedVarianceThreshold_ &&
        varAngularSpeedAroundYBodyAxis_.getVariance()< angularSpeedVarianceThreshold_ &&
        varAngularSpeedAroundZBodyAxis_.getVariance()< angularSpeedVarianceThreshold_;
  }
  else
  {
    return false;
  }

}

//-----------------------------------------------------------------------------
double ZeroVelocityEstimator::getAccelerationStd()const
{
  return std::sqrt(varAccelerationAlongXBodyAxis_.getVariance()+
                   varAccelerationAlongYBodyAxis_.getVariance()+
                   varAccelerationAlongYBodyAxis_.getVariance());
}

//-----------------------------------------------------------------------------
double ZeroVelocityEstimator::getAngularSpeedStd()const
{
  return std::sqrt(varAngularSpeedAroundXBodyAxis_.getVariance()+
                   varAngularSpeedAroundXBodyAxis_.getVariance()+
                   varAngularSpeedAroundXBodyAxis_.getVariance());
}

//-----------------------------------------------------------------------------
bool ZeroVelocityEstimator::update(const Eigen::Vector3d & accelerationSpeeds,
                                   const Eigen::Vector3d & angularSpeeds)
{
  return update(accelerationSpeeds.x(),
                accelerationSpeeds.y(),
                accelerationSpeeds.z(),
                angularSpeeds.x(),
                angularSpeeds.y(),
                angularSpeeds.z());
}

}
