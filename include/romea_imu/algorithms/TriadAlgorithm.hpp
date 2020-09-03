#ifndef _romea_TriadAttitude_hpp
#define _romea_TriadAttitude_hpp

//romea
#include <Eigen/Eigen>

namespace romea {

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

}

#endif
