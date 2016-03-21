
#include <excavaROB_truck_common/truck_function.h>


namespace excavaROB_truck_common
{

  bool transformTo (const tf::Transformer& tf, const std::string& goal_frame, const truck_msgs::Truck& truck_in,
		    const truck_msgs::Truck& truck_out, const std::string& fixed_frame="odom_combined")
  {
    truck_out = truck_in;
    ros::Time time_now = ros::Time::now();
    if (!transformPointTo (tf, door_in.header.frame_id, goal_frame, door_in.header.stamp, door_in.frame_p1, door_out.frame_p1, fixed_frame, time_now))
      return false;
    if (!transformPointTo (tf, door_in.header.frame_id, goal_frame, door_in.header.stamp, door_in.frame_p2, door_out.frame_p2, fixed_frame, time_now))
      return false;
    if (!transformPointTo (tf, door_in.header.frame_id, goal_frame, door_in.header.stamp, door_in.door_p1, door_out.door_p1, fixed_frame, time_now))
      return false;
    if (!transformPointTo (tf, door_in.header.frame_id, goal_frame, door_in.header.stamp, door_in.door_p2, door_out.door_p2, fixed_frame, time_now))
      return false;
    if (!transformPointTo (tf, door_in.header.frame_id, goal_frame, door_in.header.stamp, door_in.handle, door_out.handle, fixed_frame, time_now))
      return false;
    if (!transformVectorTo (tf, door_in.header.frame_id, goal_frame, door_in.header.stamp, door_in.travel_dir, door_out.travel_dir, fixed_frame, time_now))
      return false;

    truck_out.header.frame_id = goal_frame;
    truck_out.header.stamp = time_now;

    return true;
  };

} //@namespace excavaROB_truck_common
