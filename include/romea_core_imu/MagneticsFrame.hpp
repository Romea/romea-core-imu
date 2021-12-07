#ifndef romea_MagneticsFrame_hpp
#define romea_MagneticsFrame_hpp


//std
#include <memory>

namespace romea {

struct MagneticsFrame
{

  using Ptr =std::shared_ptr<MagneticsFrame> ;
  double magneticAlongXAxis;
  double magneticAlongYAxis;
  double magneticAlongZAxis;
};

}

#endif
