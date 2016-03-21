
#include <truck_detector/truck_detector.h>
#include <excavaROB_truck_common/truck_functions.h>

// include messages
#include <truck_msgs/Truck.h>


using namespace truck_detector;
using namespace excavaROB_truck_common;


TruckDetector::TruckDetector() : node_("~")
{
  node_.param<std::string> ("parameter_frame", parameter_frame_, "base_footprint");
  node_.param<std::string> ("fixed_frame", fixed_frame_, "odom_combined");

  // only consider points closer than this values to the robot
  node_.param ("maximun_search_radius", maximum_search_radius_, 15.0);

  node_.param ("leaf_width", leaf_width_, 0.05);

  // advertise services
  node_.param<std::string> ("input_cloud_topic", input_cloud_topic_, "/left_laser/full_scan_cloud_filtered"); ///full_cloud

  detect_srv_ = node_.advertiseService ("truck_detector", &TruckDetector::detectTruckSrv, this);

  regions_pub_ = node_.advertise<sensor_msgs::PointCloud> ("clusters_regions", 1);




  euclidean_cluster_distance_tolerance_ = 0.05;
}

bool TruckDetector::detectTruck (const truck_msgs::Truck& truck_estimated, sensor_msgs::PointCloud pointcloud,
	                         std::vector<truck_msgs::Truck>& truck_detected) const
{
  ROS_INFO ("Start detecting truck in a point cloud of size %i", (int)pointcloud.points.size());

  ros::Time ts = ros::Time::now();
  ros::Duration duration;
  ros::Duration timeout = ros::Duration().fromSec(5.0);

  // transform the PCD (Point Cloud Data) into the parameter_frame, and work there
  if (!tf_.waitForTransform (parameter_frame_, pointcloud.header.frame_id, pointcloud.header.stamp, timeout))
  {
    ROS_ERROR ("Could not transform point cloud from frame '%s' to frame '%s'at time %f.",
               pointcloud.header.frame_id.c_str(), parameter_frame_.c_str(), pointcloud.header.stamp.toSec());
    return false;
  }
  tf_.transformPointCloud (parameter_frame_, pointcloud, pointcloud);
  ROS_INFO ("PointCloud transformed to parameter frame.");

  // transform the truck message into the parameter_frame, and work there
/*  truck_msg::Truck truck_tr;
  if (!transformTo (tf_, parameter_frame_, truck, truck_tr, fixed_frame_))
  {
    ROS_ERROR ("Could not transform truck message from '%s' to '%s' at time %f.",
               truck.header.frame_id.c_str(), parameter_frame_.c_str(), truck.header.stamp.toSec());
    return false;
  }
  ROS_INFO ("Truck message transformed to parameter frame.");
*/

  geometry_msgs::PointStamped viewpoint_cloud;
  std::string laser_link = "left_rangefinder"; //TODO: to think how implement this!
  getCloudViewPoint (parameter_frame_, viewpoint_cloud, &tf_, laser_link);

  // select points whose distances are near to robot
  int nr_p = 0;
  std::vector<int> indices_in_bounds(pointcloud.points.size());
  for (unsigned int i = 0; i < pointcloud.points.size(); i++)
  {
    double dist = geometry::distances::distancePointToPoint (viewpoint_cloud.point, pointcloud.points[i]);
    if (dist < maximum_search_radius_)
      indices_in_bounds[nr_p++] = i;
  }
  indices_in_bounds.resize(nr_p);

  // NOTE: <leaves_> gets allocated internally in downsamplePointCloud() and is not deallocated on exit
  sensor_msgs::PointCloud cloud_down;
  std::vector<geometry::Leaf> leaves;
  try
  {
    geometry_msgs::Point leaf_width_xyz;
    leaf_width_xyz.x = leaf_width_xyz.y = leaf_width_xyz.z = leaf_width_;
    geometry::downsamplePointCloud (pointcloud, indices_in_bounds, cloud_down, leaf_width_xyz, leaves, -1);
    ROS_INFO ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_xyz.x, leaf_width_xyz.y, leaf_width_xyz.z, (int)cloud_down.points.size ());
  }
  catch (std::bad_alloc)
  {
    // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
    return false;
  }
  leaves.resize (0);    // dealloc memory used for the downsampling process

  //TODO: euclidean clusters!
  // split pointcloud into euclidean clusters
  std::vector<std::vector<int> > clusters;
  findEuclideanClusters (cloud_down, euclidean_cluster_distance_tolerance_, clusters);
  // sort the clusters
  sort (clusters.begin(), clusters.end(), compareRegions);
  reverse (clusters.begin(), clusters.end());

  // make a clusters regions
  sensor_msgs::PointCloud cloud_regions;
  cloud_regions.header = cloud_down.header;
  cloud_regions.points.resize (0);
  cloud_regions.channels.resize (1);
  cloud_regions.channels[0].name = "rgb";
  cloud_regions.channels[0].values.resize (0);
  for (unsigned int cr = 0; cr < clusters.size(); cr++)
  {
    float r = rand() / (RAND_MAX + 1.0);
    float g = rand() / (RAND_MAX + 1.0);
    float b = rand() / (RAND_MAX + 1.0);
    for (unsigned int i = 0; i < clusters[cr].size(); i++)
    {
      cloud_regions.points.push_back (cloud_down.points[clusters[cr][i]]);
      cloud_regions.channels[0].values.push_back (getRGB (r, g, b));
    }
  }

  regions_pub_.publish (cloud_regions);

  // select points that not represent the ground plane





  return true;
}


bool TruckDetector::detectTruckSrv (truck_detector::TrucksDetector::Request &req, truck_detector::TrucksDetector::Response &resp)
{
  ROS_INFO("Transforming truck message to fixed frame ('%s').", fixed_frame_.c_str());
  truck_msgs::Truck truck_msg;
  if (!transformTo (tf_, fixed_frame_, req.truck, truck_msg, fixed_frame_))
  {
    ROS_ERROR ("Could not transform truck message from '%s' to '%s' at time %f.",
               req.truck.header.frame_id.c_str(), fixed_frame_.c_str(), req.truck.header.stamp.toSec());
    return false;
  }


  ROS_INFO ("Truck detection waiting for pointcloud to come in on topic %s", input_cloud_topic_.c_str());
  // receive a new laser scan
  num_clouds_received_ = 0;
  ros::Subscriber cloud_sub = node_.subscribe (input_cloud_topic_, 1, &TruckDetector::cloudCallBack, this);
  ros::Duration tictoc = ros::Duration().fromSec (0.5);
  while ((int)num_clouds_received_ < 1)
    tictoc.sleep ();
  cloud_sub.shutdown();

  return detectTruck (truck_msg, pointcloud_, resp.trucks);
}


void TruckDetector::cloudCallBack (const sensor_msgs::PointCloudConstPtr& cloud)
{
  pointcloud_ = *cloud;
  ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)pointcloud_.points.size(), pointcloud_.header.frame_id.c_str(),
            (int)pointcloud_.channels.size(), geometry::getAvailableChannels (pointcloud_).c_str());
  num_clouds_received_++;
}




int main (int argc, char** argv)
{
  ros::init (argc, argv, "truck_detector_node");

  truck_detector::TruckDetector detector;

//  ros::MultiThreadedSpinner s(2);
//  s.spin();

  return (0);
}

