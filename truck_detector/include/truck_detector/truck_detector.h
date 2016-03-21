
#ifndef TRUCK_DETECTOR_H
#define TRUCK_DETECTOR_H

// ROS core
#include <ros/ros.h>

// Most of the geometric routines that contribute to the truck finding job
#include <truck_detector/geometric_helper.h>

// include service
#include <truck_detector/TrucksDetector.h>


namespace truck_detector
{

class TruckDetector
{
  public:
    TruckDetector();

    ~TruckDetector() {};

    bool detectTruckSrv (truck_detector::TrucksDetector::Request &req, truck_detector::TrucksDetector::Response &resp);

    ros::ServiceServer detect_srv_;

    ros::Publisher regions_pub_;


  private:
    bool detectTruck (const truck_msgs::Truck& truck_estimated, sensor_msgs::PointCloud pointcloud,
                      std::vector<truck_msgs::Truck>& truck_detected) const;

    void cloudCallBack (const sensor_msgs::PointCloudConstPtr& cloud);


    ros::NodeHandle node_;

    sensor_msgs::PointCloud pointcloud_;

    unsigned int num_clouds_received_;

    std::string input_cloud_topic_;

    tf::TransformListener tf_;

    std::string parameter_frame_, fixed_frame_;

    double maximum_search_radius_;

   // Parameters for the euclidean clustering/cluster rejection
    double euclidean_cluster_distance_tolerance_;

    double leaf_width_;

};  //@class TruckDetector

}  //@namespace truck_detector

#endif //TRUCK_DETECTOR_H
