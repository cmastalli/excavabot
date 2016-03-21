
#ifndef _GEOMETRY_DISTANCES_H_
#define _GEOMETRY_DISTANCES_H_


#include <math.h>

// ROS includes
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>


namespace geometry
{

  namespace distances
  {

    /** \brief Compute the euclidean distance between two points
      * \param p0 Initial point
      * \param pf Final point
      */
    inline double distancePointToPoint (const geometry_msgs::Point &p0, const geometry_msgs::Point32 &pf)
    {
      return sqrt((p0.x - pf.x) * (p0.x - pf.x) + (p0.y - pf.y) * (p0.y - pf.y) + (p0.z - pf.z) * (p0.z - pf.z));
    }

  }  //@namespace distances

}  //@namespace geometry


#endif  //_GEOMETRY_DISTANCES_H_
