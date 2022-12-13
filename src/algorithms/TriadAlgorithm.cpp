// romea
#include "romea_core_imu/algorithms/TriadAlgorithm.hpp"


namespace romea {

//--------------------------------------------------------------------
TriadAttitude::TriadAttitude():
  currentAttitude_(Eigen::Matrix3d::Identity()),
  referenceAttitude_(Eigen::Matrix3d::Identity()),
  isInitialized_(false)
{

}

//--------------------------------------------------------------------
TriadAttitude::TriadAttitude(const Eigen::Vector3d &imuAccelerations,
                                   const Eigen::Vector3d &imuMagnetics):
  currentAttitude_(Eigen::Matrix3d::Identity()),
  referenceAttitude_(Eigen::Matrix3d::Identity()),
  isInitialized_(true)

{
  init(imuAccelerations, imuMagnetics);
}

//--------------------------------------------------------------------
inline void computeAttitude(const Eigen::Vector3d & imuAccelerations,
                            const Eigen::Vector3d & imuMagnetics,
                            Eigen::Matrix3d & R)
{
  R.col(2) = imuAccelerations.normalized();
  R.col(1) = (imuAccelerations.cross(imuMagnetics)).normalized();
  R.col(0) = (R.col(1)).cross(R.col(2));
}

//--------------------------------------------------------------------
void TriadAttitude::init(const Eigen::Vector3d & imuAccelerations,
                         const Eigen::Vector3d & imuMagnetics)
{
  computeAttitude(imuAccelerations, imuMagnetics, referenceAttitude_);
  isInitialized_ = true;
}


//--------------------------------------------------------------------
Eigen::Matrix3d TriadAttitude::compute(const Eigen::Vector3d & imuAccelerations,
                                       const Eigen::Vector3d & imuMagnetics)
{
  assert(isInitialized_);
  computeAttitude(imuAccelerations, imuMagnetics, currentAttitude_);
  return currentAttitude_*referenceAttitude_.inverse();
}

//--------------------------------------------------------------------
bool TriadAttitude::isInitialized()const
{
  return isInitialized_;
}

}  // namespace romea

