#ifndef romea_RollPitchCourseFrame_hpp
#define romea_RollPitchCourseFrame_hpp

#include "RollPitchFrame.hpp"

namespace romea {

struct RollPitchCourseFrame : RollPitchFrame
{

  using Ptr =std::shared_ptr<RollPitchCourseFrame> ;
  double courseAngle;
};

}

#endif
