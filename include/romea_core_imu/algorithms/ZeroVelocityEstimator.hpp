#ifndef ROMEA_CORE_IMU_ALGORITHMS_ZEROVELOCITYESTIMATOR_HPP_
#define ROMEA_CORE_IMU_ALGORITHMS_ZEROVELOCITYESTIMATOR_HPP_

// Eigen
#include <Eigen/Eigen>

// romea
#include <romea_core_common/monitoring/OnlineVariance.hpp>

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

  void reset();

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

}  // namespace romea

#endif  // ROMEA_CORE_IMU_ALGORITHMS_ZEROVELOCITYESTIMATOR_HPP_
