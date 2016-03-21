#include <ros/ros.h>
#include <ros/node_handle.h>

#include <fstream>

#include <tf/tf.h>

// messages
#include <geometry_msgs/PoseWithCovarianceStamped.h>


using namespace std;

class PoseEKFStateRecord
{
  public:

    PoseEKFStateRecord();

    ~PoseEKFStateRecord();

    bool initialize();


  private:

    void poseEKFStateCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    void writeToDisc();


    ros::NodeHandle nh_, nh_private_;

    std::string path_name_, motion_name_;

    ros::Subscriber pose_ekf_state_sub_;

    geometry_msgs::PoseWithCovarianceStamped pose_ekf_state_msg_;

    std::vector<std::vector<double> > pose_ekf_;

    std::vector<double> t_;

    double t_start_;

    bool initialized_;

}; // @class PoseEKFStateRecord



PoseEKFStateRecord::PoseEKFStateRecord(): nh_private_("~")
{
  initialized_ = false;
}


PoseEKFStateRecord::~PoseEKFStateRecord()
{
  pose_ekf_state_sub_.shutdown();
  writeToDisc();
}


void PoseEKFStateRecord::writeToDisc()
{
  path_name_.append(motion_name_ + std::string(".txt"));

  std::ofstream outfile;
  outfile.open(path_name_.c_str());
  outfile.precision(4);
  outfile.setf(ios::fixed, ios::floatfield);
  outfile.setf(ios::left, ios::adjustfield);
  for (unsigned int j = 0; j < pose_ekf_[0].size(); j++) {
    if (j == 0)
      outfile << "t" << '\t' << "x_ekf" << '\t' << "y_ekf" << '\t' << "th_ekf" << endl;

    outfile << t_[j] << '\t';
    for (unsigned int i = 0; i < pose_ekf_.size(); i++) {
      if (i == pose_ekf_.size() - 1)
        outfile << pose_ekf_[i][j] << endl;
      else
        outfile << pose_ekf_[i][j] << '\t';
    }
  }
  outfile.close();
}


bool PoseEKFStateRecord::initialize()
{
  if (!nh_private_.getParam("path_name", path_name_)) {
    ROS_FATAL("Could not found path name from parameter server");
    return false;
  }
  if (!nh_private_.getParam("motion_name", motion_name_)) {
    ROS_FATAL("Could not found motion name from parameter server");
    return false;
  }

  pose_ekf_state_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("robot_pose_ekf/odom_combined", 1, &PoseEKFStateRecord::poseEKFStateCallback, this);
  return true;
}

void PoseEKFStateRecord::poseEKFStateCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  pose_ekf_state_msg_ = *msg;
  if (!initialized_) {
    pose_ekf_.resize(3);
    t_start_ = ros::Time::now().toSec();
    initialized_ = true;
  }

  tf::Quaternion q;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(pose_ekf_state_msg_.pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  t_.push_back(ros::Time::now().toSec() - t_start_);
  pose_ekf_[0].push_back(pose_ekf_state_msg_.pose.pose.position.x);
  pose_ekf_[1].push_back(pose_ekf_state_msg_.pose.pose.position.y);
  pose_ekf_[2].push_back(yaw);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_ekf_state_record");

  PoseEKFStateRecord pose_ekf_state_record;

  if (!pose_ekf_state_record.initialize())
  {
    ROS_ERROR("Could not initialize pose EKF state record node.");
    return -1;
  }

  ros::spin();
  return 0;
}
