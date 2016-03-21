#include <ros/ros.h>
#include <ros/node_handle.h>

#include <fstream>

#include <tf/tf.h>

// messages
#include <nav_msgs/Odometry.h>

using namespace std;

class PoseTruthStateRecord
{
  public:

    PoseTruthStateRecord();

    ~PoseTruthStateRecord();

    bool initialize();


  private:

    void poseTruthStateCallback(const nav_msgs::OdometryConstPtr& msg);

    void writeToDisc();


    ros::NodeHandle nh_, nh_private_;

    std::string path_name_, motion_name_;

    ros::Subscriber pose_truth_state_sub_;

    nav_msgs::Odometry pose_truth_state_msg_;

    std::vector<std::vector<double> > pose_truth_;

    std::vector<double> t_;

    double t_start_;

    bool initialized_;

}; // @class PoseTruthStateRecord



PoseTruthStateRecord::PoseTruthStateRecord(): nh_private_("~")
{
  initialized_ = false;
}


PoseTruthStateRecord::~PoseTruthStateRecord()
{
  pose_truth_state_sub_.shutdown();
  writeToDisc();
}


void PoseTruthStateRecord::writeToDisc()
{
  path_name_.append(motion_name_ + std::string(".txt"));

  std::ofstream outfile;
  outfile.open(path_name_.c_str());
  outfile.precision(4);
  outfile.setf(ios::fixed, ios::floatfield);
  outfile.setf(ios::left, ios::adjustfield);
  for (unsigned int j = 0; j < pose_truth_[0].size(); j++) {
    if (j == 0)
      outfile << "t" << '\t' << "x_truth" << '\t' << "y_truth" << '\t' << "th_truth" << endl;

    outfile << t_[j] << '\t';
    for (unsigned int i = 0; i < pose_truth_.size(); i++) {
      if (i == pose_truth_.size() - 1)
        outfile << pose_truth_[i][j] << endl;
      else
        outfile << pose_truth_[i][j] << '\t';
    }
  }
  outfile.close();
}


bool PoseTruthStateRecord::initialize()
{
  if (!nh_private_.getParam("path_name", path_name_)) {
    ROS_FATAL("Could not found path name from parameter server");
    return false;
  }
  if (!nh_private_.getParam("motion_name", motion_name_)) {
    ROS_FATAL("Could not found motion name from parameter server");
    return false;
  }

  pose_truth_state_sub_ = nh_.subscribe<nav_msgs::Odometry>("traction_point_pose_ground_truth", 1, &PoseTruthStateRecord::poseTruthStateCallback, this);
  return true;
}

void PoseTruthStateRecord::poseTruthStateCallback(const nav_msgs::OdometryConstPtr& msg)
{
  pose_truth_state_msg_ = *msg;
  if (!initialized_) {
    pose_truth_.resize(3);
    t_start_ = ros::Time::now().toSec();
    initialized_ = true;
  }

  tf::Quaternion q;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(pose_truth_state_msg_.pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  t_.push_back(ros::Time::now().toSec() - t_start_);
  pose_truth_[0].push_back(pose_truth_state_msg_.pose.pose.position.x);
  pose_truth_[1].push_back(pose_truth_state_msg_.pose.pose.position.y);
  pose_truth_[2].push_back(yaw);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_truth_state_record");

  PoseTruthStateRecord pose_truth_state_record;

  if (!pose_truth_state_record.initialize())
  {
    ROS_ERROR("Could not initialize pose truth state record node.");
    return -1;
  }

  ros::spin();
  return 0;
}
