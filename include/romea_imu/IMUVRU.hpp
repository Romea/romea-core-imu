#ifndef romea_IMUVRU_hpp
#define romea_IMUVRU_hpp

//romea
#include "IMU6DOF.hpp"
#include "RollPitchFrame.hpp"

//eigen
#include <Eigen/Eigen>

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
         const double & angleStd);

public:

  RollPitchFrame createFrame(const double & rollAngle,
                             const double & pitchAngle);


  double getAngleStd() const;
  double getAngleVariance() const;

private:

  const double angleStd_;
  const double angleVariance_;

};

}

#endif
