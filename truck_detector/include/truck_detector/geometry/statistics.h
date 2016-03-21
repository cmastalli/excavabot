
#ifndef _GEOMETRY_STATISTICS_H_
#define _GEOMETRY_STATISTICS_H_

// ROS includes
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

#include <cfloat>

namespace geometry
{
  namespace statistics
  {
    /** \brief Get the minimum and maximum values on each of the 3 (x,y,z) dimensions in a given pointcloud,
      *        without considering points outside of a distance threshold from the laser origin
      * \param points the point clouddata message
      * \param indices the point cloud indices to use
      * \param minP the resultant minimum bounds
      * \param maxP the resultant maximum bounds
      * \param c_idx the index of the channel holding distance information
      * \param cut_distance a maximum admissible distance threshold for points from the laser origin
      */
    inline void getMinMax (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, geometry_msgs::Point32 &minP, geometry_msgs::Point32 &maxP,
                           int c_idx, double cut_distance)
    {
      minP.x = minP.y = minP.z = FLT_MAX;
      maxP.x = maxP.y = maxP.z = -FLT_MAX;

      for (unsigned int i = 0; i < indices.size(); i++)
      {
	if (c_idx != -1 && points.channels[c_idx].values[indices.at (i)] > cut_distance)
	  continue;

	minP.x = (points.points[i].x < minP.x) ? points.points[i].x : minP.x;
	minP.y = (points.points[i].y < minP.y) ? points.points[i].y : minP.y;
	minP.z = (points.points[i].z < minP.z) ? points.points[i].z : minP.z;

	maxP.x = (points.points[i].x > maxP.x) ? points.points[i].x : maxP.x;
	maxP.y = (points.points[i].y > maxP.y) ? points.points[i].y : maxP.y;
	maxP.z = (points.points[i].z > maxP.z) ? points.points[i].z : maxP.z;
      }
    } //@function getMinMax

  } //@namespace statistics

} //@namespace geometry

#endif //_GEOMETRY_STATISTICS_H_
