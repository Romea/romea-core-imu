#include "romea_core_imu/IMUAHRS.hpp"
#include <romea_core_common/math/EulerAngles.hpp>

namespace romea {



//--------------------------------------------------------------------
IMUAHRS::IMUAHRS(const double &rate,
                 const double &accelerationNoiseDensity,
                 const double &accelerationBiaisStatibilityStd,
                 const double &accelerationRange,
                 const double &angularSpeedNoiseDensity,
                 const double &angularSpeedBiaisStatibilityStd,
                 const double &angularSpeedRange,
                 const double &magneticNoiseDenity,
                 const double &magneticBiaisStatibilityStd,
                 const double &magneticRange,
                 const double &angleStd,
                 const Eigen::Affine3d &bodyPose):
  IMU9DOF(rate,
          accelerationNoiseDensity,
          accelerationBiaisStatibilityStd,
          accelerationRange,
          angularSpeedNoiseDensity,
          angularSpeedBiaisStatibilityStd,
          angularSpeedRange,
          magneticNoiseDenity,
          magneticBiaisStatibilityStd,
          magneticRange,
          bodyPose),
  angleStd_(angleStd),
  angleVariance_(angleStd*angleStd)
{

}

//--------------------------------------------------------------------
RollPitchCourseFrame IMUAHRS::createFrame(const double & rollAngle,
                                          const double & pitchAngle,
                                          const double & courseAngle)
{
  Eigen::Vector3d eulerAngles(rollAngle,pitchAngle,courseAngle);
  eulerAngles = rotation3DToEulerAngles(eulerAnglesToRotation3D(eulerAngles));

// C++11
  RollPitchCourseFrame frame;
  frame.rollAngle=betweenMinusPiAndPi(eulerAngles.x());
  frame.pitchAngle=betweenMinusPiAndPi(eulerAngles.y());
  frame.courseAngle=eulerAngles.z();
  return frame;

// C++17
// return {{eulerAngles.x(),eulerAngles.y()},eulerAngles.z()};

}

//--------------------------------------------------------------------

double IMUAHRS::getAngleStd() const
{
  return angleStd_;
}

//--------------------------------------------------------------------
double IMUAHRS::getAngleVariance() const
{
  return angleVariance_;
}



}
