#include "romea_imu/IMUVRU.hpp"
#include <romea_common/math/EulerAngles.hpp>

namespace romea {

//--------------------------------------------------------------------
IMUVRU::IMUVRU(const double & rate,
               const double & accelerationNoiseDensity,
               const double & accelerationBiasStatibilityStd,
               const double & accelerationRange,
               const double & angularSpeedNoiseDensity,
               const double & angularSpeedBiasStatibilityStd,
               const double & angularSpeedRange,
               const double & angleStd):
  IMU6DOF(rate,
          accelerationNoiseDensity,
          accelerationBiasStatibilityStd,
          accelerationRange,
          angularSpeedNoiseDensity,
          angularSpeedBiasStatibilityStd,
          angularSpeedRange),
  angleStd_(angleStd),
  angleVariance_(angleStd*angleStd)
{

}

//--------------------------------------------------------------------
RollPitchFrame IMUVRU::createFrame(const double & rollAngle,
                                   const double & pitchAngle)
{
  Eigen::Vector3d eulerAngles(rollAngle,pitchAngle,0);
  eulerAngles = rotation3DToEulerAngles(eulerAnglesToRotation3D(eulerAngles));
  return {eulerAngles.x(),eulerAngles.y()};
}

//--------------------------------------------------------------------

double IMUVRU::getAngleStd() const
{
  return angleStd_;
}

//--------------------------------------------------------------------
double IMUVRU::getAngleVariance() const
{
  return angleVariance_;
}



}
