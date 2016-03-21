#include <ros/ros.h>
#include <ros/node_handle.h>

#include <fstream>

#include <tf/tf.h>

// messages
#include <nav_msgs/Odometry.h>

using namespace std;

class PoseOdomStateRecord
{
  public:

    PoseOdomStateRecord();

    ~PoseOdomStateRecord();

    bool initialize();


  private:

    void poseOdomStateCallback(const nav_msgs::OdometryConstPtr& msg);

    void writeToDisc();


    ros::NodeHandle nh_, nh_private_;

    std::string path_name_, motion_name_;

    ros::Subscriber pose_odom_state_sub_;

    nav_msgs::Odometry pose_odom_state_msg_;

    std::vector<std::vector<double> > pose_odom_;

    std::vector<double> t_;

    double t_start_;

    bool initialized_;

}; // @class PoseOdomStateRecord



PoseOdomStateRecord::PoseOdomStateRecord(): nh_private_("~")
{
  initialized_ = false;
}


PoseOdomStateRecord::~PoseOdomStateRecord()
{
  pose_odom_state_sub_.shutdown();
  writeToDisc();
}


void PoseOdomStateRecord::writeToDisc()
{
  path_name_.append(motion_name_ + std::string(".txt"));

  std::ofstream outfile;
  outfile.open(path_name_.c_str());
  outfile.precision(4);
  outfile.setf(ios::fixed, ios::floatfield);
  outfile.setf(ios::left, ios::adjustfield);
  for (unsigned int j = 0; j < pose_odom_[0].size(); j++) {
    if (j == 0)
      outfile << "t" << '\t' <<"x_odom" << '\t' << "y_odom" << '\t' << "th_odom" << endl;

    outfile << t_[j] << '\t';
    for (unsigned int i = 0; i < pose_odom_.size(); i++) {
      if (i == pose_odom_.size() - 1)
        outfile << pose_odom_[i][j] << endl;
      else
        outfile << pose_odom_[i][j] << '\t';
    }
  }
  outfile.close();
}


bool PoseOdomStateRecord::initialize()
{
  if (!nh_private_.getParam("path_name", path_name_)) {
    ROS_FATAL("Could not found path name from parameter server");
    return false;
  }
  if (!nh_private_.getParam("motion_name", motion_name_)) {
    ROS_FATAL("Could not found motion name from parameter server");
    return false;
  }

  pose_odom_state_sub_ = nh_.subscribe<nav_msgs::Odometry>("base_odometry/odom", 1, &PoseOdomStateRecord::poseOdomStateCallback, this);
  return true;
}

void PoseOdomStateRecord::poseOdomStateCallback(const nav_msgs::OdometryConstPtr& msg)
{
  pose_odom_state_msg_ = *msg;
  if (!initialized_) {
    pose_odom_.resize(3);
    t_start_ = ros::Time::now().toSec();
    initialized_ = true;
  }

  t_.push_back(ros::Time::now().toSec() - t_start_);
  tf::Quaternion q;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(pose_odom_state_msg_.pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  pose_odom_[0].push_back(pose_odom_state_msg_.pose.pose.position.x);
  pose_odom_[1].push_back(pose_odom_state_msg_.pose.pose.position.y);
  pose_odom_[2].push_back(yaw);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_odom_state_record");

  PoseOdomStateRecord pose_odom_state_record;

  if (!pose_odom_state_record.initialize())
  {
    ROS_ERROR("Could not initialize pose odom state record node.");
    return -1;
  }

  ros::spin();
  return 0;
}
