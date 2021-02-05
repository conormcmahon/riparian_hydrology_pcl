
#ifndef POINT_CHANNEL_
#define POINT_CHANNEL_
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_representation.h>

namespace pcl{

struct PointChannel
  {
    PCL_ADD_POINT4D                     // Macro quad-word XYZ
    int point_id;                       // Unique point ID assigned within entire flow system
    int stream_order;                   // Strahler Stream order
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // Ensure proper alignment
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointChannel,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (int, stream_order, stream_order)
)

namespace pcl {
template <>
class DefaultPointRepresentation<PointChannel> : public PointRepresentation<PointChannel>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 2;
  }
  
  virtual void
  copyToFloatArray (const PointChannel &p, float * out) const
  {
    out[0] = p.x;
    out[1] = p.y;
  }
};
}
#endif // POINT_CHANNEL_
