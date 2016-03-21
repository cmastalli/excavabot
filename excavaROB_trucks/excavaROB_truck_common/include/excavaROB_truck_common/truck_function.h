#ifndef TRUCK_FUNCTIONS_H
#define TRUCK_FUNCTIONS_H

#include <truck_msgs/Truck.h>
#include <tf/tf.h>

namespace excavaROB_truck_common
{
  bool transformTo (const tf::Transformer& tf, const std::string& goal_frame, const truck_msgs::Truck& truck_in,
		    const truck_msgs::Truck& truck_out, const std::string& fixed_frame="odom_combined");

  bool transformPointTo (const tf::Transformer& tf, const std::string& source_frame, const std::string& goal_frame, const ros::Time& time_source,
                         const geometry_msgs::Point32& point_in, geometry_msgs::Point32& point_out, const std::string& fixed_frame, const ros::Time& time_goal);

} //@namespace excavaROB_truck_common

#endif //TRUCK_FUNCTIONS_H
