#ifndef romea_IMUAHRS_hpp
#define romea_IMUAHRS_hpp

//romea
#include "IMU9DOF.hpp"
#include "RollPitchCourseFrame.hpp"

//eigen
#include <Eigen/Eigen>

namespace romea {

class IMUAHRS : public IMU9DOF
{

public :

  IMUAHRS(const double & rate,
          const double & accelerationNoiseDensity,
          const double & accelerationBiaisStatibilityStd,
          const double & accelerationRange,
          const double & angularSpeedNoiseDensity,
          const double & angularSpeedBiaisStatibilityStd,
          const double & angularSpeedRange,
          const double & magneticNoiseDenity,
          const double & magneticBiaisStatibilityStd,
          const double & magneticRange,
          const double & angleStd,
          const Eigen::Affine3d & bodyPose = Eigen::Affine3d::Identity());

public:

  RollPitchCourseFrame createFrame(const double & rollAngle,
                                   const double & pitchAngle,
                                   const double & courseAngle);


  double getAngleStd() const;
  double getAngleVariance() const;

private:

  const double angleStd_;
  const double angleVariance_;

};

}

#endif
