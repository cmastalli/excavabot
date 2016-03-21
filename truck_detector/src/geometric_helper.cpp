
#include <truck_detector/geometric_helper.h>


void getCloudViewPoint (const std::string cloud_frame, geometry_msgs::PointStamped &viewpoint_cloud,
                        const tf::TransformListener *tf, const std::string laser_link)
{
  // figure out the viewpoint value in the point cloud frame
  geometry_msgs::PointStamped viewpoint_laser;
  viewpoint_laser.header.frame_id = laser_link;
  // set to zero the viewpoint of laser link
  viewpoint_laser.point.x = viewpoint_laser.point.y = viewpoint_laser.point.z = 0.0;

  try
  {
    tf->transformPoint(cloud_frame, viewpoint_laser, viewpoint_cloud);
    ROS_INFO ("Cloud viewpoint in frame %s is: [%g, %g, %g].", cloud_frame.c_str(),
              viewpoint_cloud.point.x, viewpoint_laser.point.y, viewpoint_laser.point.z);
  }
  catch (tf::ConnectivityException)
  {
    ROS_WARN ("Could not transform a point from frame %s to frame %s!", viewpoint_laser.header.frame_id.c_str (), cloud_frame.c_str ());
    // Default to 0.0, 0.0, 0.0
    viewpoint_cloud.point.x = 0.0; viewpoint_cloud.point.y = 0.0; viewpoint_cloud.point.z = 0.0;
  }
} //@function getCloudViewPoint


void findEuclideanClusters (const sensor_msgs::PointCloud &points, double tolerance, std::vector<std::vector<int> > &clusters, unsigned int min_points_per_cluster)
{
  // create a tree for these points
  kdtree::KdTree* tree = new kdtree::KdTreeANN (points);

  int nr_points = points.points.size();
  // create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (nr_points, false);

  std::vector<int> k_indices;
  std::vector<float> k_distances;
  // process all points
  for (int i = 0; i < nr_points; i++)
  {
    // if we already processed this point continue with the next one
    if (processed[i])
      continue;
    // now we will process this point
    processed[i] = true;

    // create the query queue
    std::vector<int> seed_queue;
    // push the starting point in the vector
    seed_queue.push_back (i);
    int sq_idx = 0;

    while (sq_idx < (int)seed_queue.size())
    {
      // search for seed queue index
      tree->radiusSearch (seed_queue.at (sq_idx), tolerance, k_indices, k_distances);

      for (unsigned int j = i; j < k_indices.size(); j++)  // k_indices[0] should be sq_idx
      {
	// continue with the next points if we already processed
	if (processed.at (k_indices[j]))
	  continue;
	// now we will process this point
	processed.at (k_indices[j]) = true;
	// push the index of the point in the vector
	seed_queue.push_back (k_indices[j]);
      }
      sq_idx++;
    }

    // add to the cluster if this queue is satisfactory
    if (seed_queue.size() >= min_points_per_cluster)
    {
//      std::vector<int> r (seed_queue.size());
//      for (unsigned int j = 0; j < seed_queue.size(); j++)
//        r[j] = seed_queue[j]

      sort (seed_queue.begin(), seed_queue.end());
      seed_queue.erase (unique (seed_queue.begin(), seed_queue.end()), seed_queue.end());

      clusters.push_back (seed_queue);
    }
  }

  // destroy the tree
  delete tree;
} //@function findEuclideanClusters


