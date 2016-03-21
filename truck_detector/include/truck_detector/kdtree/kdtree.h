
#ifndef _KDTREE_KDTREE_H_
#define _KDTREE_KDTREE_H_

#include <sensor_msgs/PointCloud.h>

namespace kdtree
{
  class KdTree
  {
    public:
      /** \brief Empty constructor for KdTree. Sets some internal values to their defaults.
        */
      KdTree () 
      {
        epsilon_ = 0.0; // default error bound value
      }

      /** \brief Constructor for KdTree.
        * \param points the ROS point cloud data array
        */
      KdTree (const sensor_msgs::PointCloud &points);

      /** \brief Destructor for KdTree. Deletes all allocated data arrays and destroys the kd-tree structures. */
      virtual ~KdTree () {}

      virtual bool radiusSearch (int index, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances,
				 int max_nn = INT_MAX) = 0;


    protected:
      /** \brief Epsilon precision (error bound) for nearest neighbors searches */
      double epsilon_;

  }; //@class kdtree

} //@namespace kdtree

#endif //_KDTREE_KDTREE_H_
