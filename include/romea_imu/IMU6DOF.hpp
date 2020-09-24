#ifndef romea_IMU6DOF_hpp
#define romea_IMU6DOF_hpp

//romea
#include "AccelerationsFrame.hpp"
#include "AngularSpeedsFrame.hpp"

//eigen
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace romea {

class IMU6DOF
{

public :

  IMU6DOF(const double & rate,
          const double & accelerationNoiseDensity,
          const double & accelerationBiasStatibilityStd,
          const double & accelerationRange,
          const double & angularSpeedNoiseDensity,
          const double & angularSpeedBiasStatibilityStd,
          const double & angularSpeedRange);

  virtual ~IMU6DOF()=default;

public:

  AccelerationsFrame createAccelerationsFrame(const double & accelerationAlongXAxis,
                                              const double & accelerationAlongYAxis,
                                              const double & accelerationAlongZAxis);

  AngularSpeedsFrame createAngularSpeedsFrame(const double & angularSpeedAroundXAxis,
                                              const double & angularSpeedAroundYAxis,
                                              const double & angularSpeedAroundZAxis);

public :

  const double & getRate() const;

  double getAccelerationStd() const;
  double getAccelerationVariance() const;
  const double & getAccelerationRange() const;
  const double & getAccelerationNoiseDensity()const;
  const double & getAccelerationBiasStatibilityStd()const;

  double getAngularSpeedStd() const;
  double getAngularSpeedVariance() const;
  const double & getAngularSpeedRange() const;
  const double & getAngularSpeedNoiseDensity()const;
  const double & getAngularSpeedBiasStatibilityStd()const;

  void setBodyPose(const Eigen::Affine3d & rigidTransformation);
  const Eigen::Affine3d & getBodyPose()const;

protected :

  double rate_;

  double accelerationNoiseDensity_;
  double accelerationBiasStatibilityStd_;
  double accelerationRange_;

  double angularSpeedNoiseDensity_;
  double angularSpeedBiasStatibilityStd_;
  double angularSpeedRange_;

  Eigen::Affine3d rigidTransformation_;

};

}

#endif
