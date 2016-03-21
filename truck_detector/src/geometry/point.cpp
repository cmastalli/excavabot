#include <cfloat>

#include <truck_detector/geometry/point.h>
#include <truck_detector/geometry/statistics.h>


namespace geometry
{

  void downsamplePointCloud (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, sensor_msgs::PointCloud &points_down,
                             geometry_msgs::Point leaf_size, std::vector<Leaf> &leaves, int d_idx, double cut_distance)
  {
    if (d_idx == -1) //nonexistant channel of distance data
      cut_distance = DBL_MAX;

    // copy the header and allocate enough space for points
    points_down.header = points.header;
    points_down.points.resize (points.points.size());

    geometry_msgs::Point32 min_p, max_p, min_b, max_b, div_b;
    geometry::statistics::getMinMax (points, indices, min_p, max_p, d_idx, cut_distance);

    // compute the minimum and maximum bounding box values
    min_b.x = (int)(floor (min_p.x / leaf_size.x));
    max_b.x = (int)(floor (max_p.x / leaf_size.x));

    min_b.y = (int)(floor (min_p.y / leaf_size.y));
    max_b.y = (int)(floor (max_p.y / leaf_size.y));

    min_b.z = (int)(floor (min_p.z / leaf_size.z));
    max_b.z = (int)(floor (max_p.z / leaf_size.z));

    // compute the number of divisions per axis
    div_b.x = (int)(max_b.x - min_b.x + 1);
    div_b.y = (int)(max_b.y - min_b.y + 1);
    div_b.z = (int)(max_b.z - min_b.z + 1);

    try
    {
      if (leaves.capacity() < div_b.x * div_b.y * div_b.z)
	leaves.reserve (div_b.x * div_b.y * div_b.z);
      leaves.resize (div_b.x * div_b.y * div_b.z);
    }
    catch (std::bad_alloc)
    {
      ROS_ERROR ("Failed while attempting to allocate a vector of %f (%g x %g x %g) leaf elements (%f bytes total)", div_b.x * div_b.y * div_b.z,
                 div_b.x, div_b.y, div_b.z, div_b.x * div_b.y * div_b.z * sizeof (Leaf));
    }

    unsigned long leaves_size = div_b.x * div_b.y * div_b.z;
    if (leaves_size != leaves.size())
      ROS_ERROR ("That's odd: %lu != %zu", leaves_size, leaves.size());
    for (unsigned int cl = 0; cl < leaves_size; cl++)
    {
      if (leaves[cl].nr_points > 0)
      {
	leaves[cl].centroid_x = leaves[cl].centroid_y = leaves[cl].centroid_z = 0.0;
	leaves[cl].nr_points = 0;
      }
    }

    // go over all points and insert them into the right leaf
    for (unsigned int cp = 0; cp < indices.size(); cp++)
    {
      if (d_idx != -1 && points.channels[d_idx].values[indices.at (cp)] > cut_distance)
	continue;

      int i = (int)(floor (points.points[indices.at (cp)].x / leaf_size.x));
      int j = (int)(floor (points.points[indices.at (cp)].y / leaf_size.y));
      int k = (int)(floor (points.points[indices.at (cp)].z / leaf_size.z));

      int idx = ((k - min_b.z) * div_b.y * div_b.z) + ((j - min_b.y) * div_b.x) + (i - min_b.x);
      leaves[idx].centroid_x += points.points[indices.at (cp)].x;
      leaves[idx].centroid_y += points.points[indices.at (cp)].y;
      leaves[idx].centroid_z += points.points[indices.at (cp)].z;
      leaves[idx].nr_points++;
    }

    // go over all leaves and compute centroids
    int nr_p =0;
    for (unsigned int cl = 0; cl < leaves_size; cl++)
    {
      if (leaves[cl].nr_points > 0)
      {
	points_down.points[nr_p].x = leaves[cl].centroid_x / leaves[cl].nr_points;
	points_down.points[nr_p].y = leaves[cl].centroid_y / leaves[cl].nr_points;
	points_down.points[nr_p].z = leaves[cl].centroid_z / leaves[cl].nr_points;
	nr_p++;
      }
    }
    points_down.points.resize (nr_p);

  } //@function downsamplePointCloud


  std::string getAvailableChannels (const sensor_msgs::PointCloud &cloud)
  {
    std::string result;
    if (cloud.channels.size() == 0)
      return result;

    for (unsigned int i = 0; i < cloud.channels.size(); i++)
    {
      std::string index;
      if (i == cloud.channels.size() - 1)
	index = cloud.channels[i].name;
      else
	index = cloud.channels[i].name + " ";
      result += index;
    }

    return result;
  } //@function getAvailableChannels

} //@namespace geometry
