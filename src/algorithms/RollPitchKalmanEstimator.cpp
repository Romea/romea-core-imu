//Eigen
#include <Eigen/LU>

//romea
#include "romea_imu/algorithms/RollPitchKalmanEstimator.hpp"
#include <romea_common/math/Algorithm.hpp>
#include <romea_common/math/EulerAngles.hpp>

namespace romea {

namespace {
const double GRAVITY = -9.81;
}

//--------------------------------------------------------------------
RollPitchKalmanEstimator::RollPitchKalmanEstimator():
  X_(Eigen::Vector2d::Zero()),
  P_(Eigen::Matrix2d::Zero()),
  F_(Eigen::MatrixXd::Zero(2,3)),
  H_(Eigen::MatrixXd::Zero(3,2)),
  Inn_(Eigen::Vector3d::Zero()),
  InnCov_(Eigen::Matrix3d ::Zero()),
  K_(Eigen::MatrixXd::Zero(2,3)),
  isInitialized_(false),
  previousDuration_(Duration::zero())

{

}


//--------------------------------------------------------------------
void RollPitchKalmanEstimator::init(const Duration & duration,
                                    const Eigen::Vector3d & imuAccelerations,
                                    const double & /*imuAccelerationsVar*/)
{
  //initialization
  X_(0)=computeRoll(imuAccelerations);
  X_(1)=computePitch(imuAccelerations);
  P_ = Eigen::Matrix2d::Identity();
  previousDuration_=duration;
  isInitialized_=true;
}

//--------------------------------------------------------------------
void RollPitchKalmanEstimator::update(const Duration & duration,
                                      const Eigen::Vector3d & imuAccelerations,
                                      const double & imuAccelerationsVar,
                                      const Eigen::Vector3d & imuAngularSpeeds,
                                      const double & imuAngularSpeedsVar)
{

  assert(isInitialized_);

  //prediction
  double & roll =X_[0];
  double & pitch =X_[1];

  double dt = durationToSecond(duration-previousDuration_);

  F_.row(0)<< 1 , std::sin(roll)*std::tan(pitch), std::cos(roll)*std::tan(pitch);
  F_.row(1)<< 0 , std::cos(roll)               , -std::sin(roll);

  X_+= F_*imuAngularSpeeds*dt;
  P_ += F_*F_.transpose()*imuAngularSpeedsVar*dt*dt;


  //update
  H_.row(0)<< 0                             ,   std::cos(pitch);
  H_.row(1)<<  -std::cos(roll)*std::cos(pitch), std::sin(roll)*std::sin(pitch);
  H_.row(2)<<  std::sin(roll)*std::cos(pitch),  std::cos(roll)*std::sin(pitch);

  Inn_ <<  -std::sin(pitch), std::sin(roll)*std::cos(pitch), std::cos(roll)*std::cos(pitch);
  Inn_ = imuAccelerations + Inn_* 9.81;

  InnCov_ = H_*P_*H_.transpose() + Eigen::Matrix3d::Identity()*
            (imuAccelerationsVar+Inn_.squaredNorm());

  K_ =  P_ *H_.transpose() * InnCov_.inverse();
  X_ += K_*Inn_;
  P_ -= K_*H_*P_;

  if(X_[1]>M_PI/2)
  {
    X_[0]+= M_PI;
    X_[1] = M_PI+X_[1];
  }

  if(X_[1]<-M_PI/2)
  {
    X_[0]+= M_PI;
    X_[1] =-M_PI-X_[1];
  }

  X_[0]=betweenMinusPiAndPi(X_[0]);

  previousDuration_=duration;

}

//--------------------------------------------------------------------
double RollPitchKalmanEstimator::computeRoll(const Eigen::Vector3d & imuAccelerations)
{

  double normXY = std::sqrt(imuAccelerations.z()*imuAccelerations.z() +
                            0.01*imuAccelerations.x()*imuAccelerations.x());

  return sign(imuAccelerations.z())*std::atan2(imuAccelerations.y(),normXY);
}

//--------------------------------------------------------------------
double RollPitchKalmanEstimator::computePitch(const Eigen::Vector3d & imuAccelerations)
{
  double normYZ = std::sqrt(imuAccelerations.y()*imuAccelerations.y()+
                            imuAccelerations.z()*imuAccelerations.z());

  return std::atan2(imuAccelerations.x(),normYZ);
}

//--------------------------------------------------------------------
double RollPitchKalmanEstimator::getRoll()const
{
  assert(isInitialized_);
  return X_[0];
}

//--------------------------------------------------------------------
double RollPitchKalmanEstimator::getPitch()const
{
  assert(isInitialized_);
  return X_[1];

}

//--------------------------------------------------------------------
double RollPitchKalmanEstimator::getRollVariance()const
{
  assert(isInitialized_);
  return P_(0,0);
}

//--------------------------------------------------------------------
double RollPitchKalmanEstimator::getPitchVariance()const
{
  assert(isInitialized_);
  return P_(1,1);
}

//--------------------------------------------------------------------
Eigen::Matrix2d RollPitchKalmanEstimator::getRollPitchCovariance() const
{
  assert(isInitialized_);
  return P_;
}

//--------------------------------------------------------------------
bool RollPitchKalmanEstimator::isInitialized()
{
  return isInitialized_;
}



}

