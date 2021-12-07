#ifndef romea_RollPitchFrame_hpp
#define romea_RollPitchFrame_hpp

//eigen
#include <Eigen/Eigen>

//std
#include <memory>

namespace romea {


struct RollPitchFrame
{
  using Ptr=std::shared_ptr<RollPitchFrame> ;

  double rollAngle;
  double pitchAngle;
};

}

#endif
