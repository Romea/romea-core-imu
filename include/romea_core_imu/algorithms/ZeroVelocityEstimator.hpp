#ifndef romea_ZeroVelocityEstimator_hpp
#define romea_ZeroVelocityEstimator_hpp

//romea
#include <romea_core_common/monitoring/OnlineVariance.hpp>

//Eigen
#include <Eigen/Eigen>

namespace romea
{

class ZeroVelocityEstimator
{

public:

  ZeroVelocityEstimator(const double &imuRate,
                        const double & accelerationSpeedStd,
                        const double & angularSpeedStd);

  void init(const double & accelerationSpeedStd,
            const double & angularSpeedStd);


  bool update(const double & accelerationAlongXBodyAxis,
              const double & accelerationAlongYBodyAxis,
              const double & accelerationAlongZBodyAxis,
              const double & angularSpeedAroundXBodyAxis,
              const double & angularSpeedAroundYBodyAxis,
              const double & angularSpeedAroundZBodyAxis);

  bool update(const Eigen::Vector3d & accelerationSpeeds,
              const Eigen::Vector3d & angularSpeeds);

  double getAccelerationStd()const;

  double getAngularSpeedStd()const;


private:

  double accelerationVarianceThreshold_;
  double angularSpeedVarianceThreshold_;

  OnlineVariance varAccelerationAlongXBodyAxis_;
  OnlineVariance varAccelerationAlongYBodyAxis_;
  OnlineVariance varAccelerationAlongZBodyAxis_;
  OnlineVariance varAngularSpeedAroundXBodyAxis_;
  OnlineVariance varAngularSpeedAroundYBodyAxis_;
  OnlineVariance varAngularSpeedAroundZBodyAxis_;

};

}

#endif
