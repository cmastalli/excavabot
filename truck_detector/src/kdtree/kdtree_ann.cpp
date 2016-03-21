
#include <truck_detector/kdtree/kdtree_ann.h>

namespace kdtree
{

  int KdTreeANN::convertCloudToArray (const sensor_msgs::PointCloud &cloud)
  {
    if (cloud.points.size() == 0)
    {
      m_lock_.lock ();
      points_ = NULL;
      m_lock_.unlock ();
      return 0;
    }

    m_lock_.lock ();
    points_ = annAllocPts (cloud.points.size(), 3); // default dimension (xyz = 3)

    for (unsigned int i = 0; i < cloud.points.size(); i++)
    {
      points_[i][0] = cloud.points[i].x;
      points_[i][1] = cloud.points[i].y;
      points_[i][2] = cloud.points[i].z;
    }
    m_lock_.unlock ();

    return cloud.points.size();
  } //@function convertCloudToArray

} //@namespace kdtree
