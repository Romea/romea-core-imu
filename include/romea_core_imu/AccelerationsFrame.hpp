#ifndef romea_AccelerationsFrame_hpp
#define romea_AccelerationsFrame_hpp

//std
#include <memory>


namespace romea {

struct AccelerationsFrame
{

  using Ptr=std::shared_ptr<AccelerationsFrame> ;

  double accelerationAlongXAxis;
  double accelerationAlongYAxis;
  double accelerationAlongZAxis;
};

}

#endif
