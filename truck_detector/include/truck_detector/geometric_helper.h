
#ifndef _TRUCK_GEOMETRIC_HELPER_H_
#define _TRUCK_GEOMETRIC_HELPER_H_

#include <vector>

//ROS include
#include <ros/ros.h>

// Messages includes
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>

// PointCloud Mapping includes
#include <truck_detector/geometry/point.h>
#include <truck_detector/geometry/statistics.h>
#include <truck_detector/kdtree/kdtree_ann.h>

#include <truck_detector/geometry/distances.h>

#include <tf/transform_listener.h>


/** \brief Compare the size of two regions 
  * \param reg_a The first region
  * \param reg_b The second region
  */
inline bool compareRegions (const std::vector<int> &reg_a, const std::vector<int> &reg_b)
{
  return (reg_a.size() < reg_b.size());
}

inline double getRGB (float r, float g, float b)
{
  int res = (int(r * 255) << 16) | (int(g * 255) << 8) | (int(b * 255));
  double rgb = *(float*)(&res);

  return rgb;
}

/** \brief Get the viewpoint of Point Cloud Data (PCD)
  * \param cloud_frame PCD frame target
  * \param viewpoint_cloud a pointer to view point of the PCD
  * \param tf transform listener
  * \param laser_link The link of the sense data
  */
void getCloudViewPoint(const std::string cloud_frame, geometry_msgs::PointStamped &viewpoint_cloud,
                       const tf::TransformListener *tf, const std::string laser_link);


/** \brief Decompose a region of space into clusters based on the euclidean distance between points, and the normal
  *        angular deviation TODO: to modify
  * \NOTE: assumes normalized point normals !
  * \param points pointer to the point cloud message
  * \param indices pointer to a list of point indices
  * \param tolerance the spatial tolerance as a measure in the L2 Euclidean space
  * \param clusters the resultant clusters
  * \param nx_idx the index of the channel containing the X component of the normal
  * \param ny_idx the index of the channel containing the Y component of the normal
  * \param nz_idx the index of the channel containing the Z component of the normal
  * \param eps_angle the maximum allowed difference between normals in degrees for cluster/region growing
  * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
  */
void findEuclideanClusters (const sensor_msgs::PointCloud &points, double tolerance, std::vector<std::vector<int> > &clusters,
			    unsigned int min_points_per_cluster = 1);



#endif  //_TRUCK_GEOMETRIC_HELPER_H_
