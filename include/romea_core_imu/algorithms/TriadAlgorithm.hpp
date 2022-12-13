#ifndef ROMEA_CORE_IMU_ALGORITHMS_TRIADALGORITHM_HPP_
#define ROMEA_CORE_IMU_ALGORITHMS_TRIADALGORITHM_HPP_

// Eigen
#include <Eigen/Eigen>

namespace romea 
{

class TriadAttitude{
public :

  TriadAttitude();

  TriadAttitude(const Eigen::Vector3d & imuAccelerations,
                const Eigen::Vector3d & imuMagnetics);

public :

  void init(const Eigen::Vector3d &imuAccelerations,
            const Eigen::Vector3d &imuMagnetics);

  Eigen::Matrix3d compute(const Eigen::Vector3d & imuAccelerations,
                          const Eigen::Vector3d & imuMagnetics);

  bool isInitialized()const;

private:
  Eigen::Matrix3d currentAttitude_;
  Eigen::Matrix3d referenceAttitude_;
  bool isInitialized_;
};

}  // namespace romea

#endif  // ROMEA_CORE_IMU_ALGORITHMS_TRIADALGORITHM_HPP_
