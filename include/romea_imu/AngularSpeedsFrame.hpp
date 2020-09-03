#ifndef romea_AngularSpeedsFrame_hpp
#define romea_AngularSpeedsFrame_hpp

//std
#include <memory>

namespace romea {


struct AngularSpeedsFrame
{

  using Ptr=std::shared_ptr<AngularSpeedsFrame> ;

  double angularSpeedAlongXAxis;
  double angularSpeedAlongYAxis;
  double angularSpeedAlongZAxis;
};

}

#endif
