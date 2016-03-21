
#ifndef _GEOMETRY_POINT_H_
#define _GEOMETRY_POINT_H_

#include <cfloat>

// ROS includes
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>


namespace geometry
{
  /** \brief Simple leaf (3d box) structure) holding a centroid and the number of points in the leaf */
  struct Leaf
  {
     float centroid_x, centroid_y, centroid_z;
     unsigned short nr_points;
  }; //@struct Leaf

  /** \brief Downsample a Point Cloud using a voxelized grid approach
    * \param points a pointer to the pointcloud message
    * \param indices a pointer to indices of pointcloud message
    * \param points_down the resultant downsampled pointcloud
    * \param leaf_size the voxel leaf dimensions
    * \param leaves a vector of already existing leaves (empty for the first call)
    * \param d_idx the index of the channel providing distance data (set to -1 if nonexistant)
    * \param cut_distance the maximum admissible distance of a point from the viewpoint (default: FLT_MAX)
    */
  void downsamplePointCloud (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, sensor_msgs::PointCloud &points_down,
                             geometry_msgs::Point leaf_size, std::vector<Leaf> &leaves, int d_idx, double cut_distance = DBL_MAX);

  /** \brief Get the names of available channels in a Point Cloud Data
    * \param cloud a pointer to the pointcloud message
    * \return a names of available channels
    */
  std::string getAvailableChannels (const sensor_msgs::PointCloud &cloud);



} //@namespace geometry

#endif //_GEOMETRY_POINT_H_
