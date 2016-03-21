#include <ros/ros.h>
#include <ros/node_handle.h>

#include <fstream>

#include <tf/tf.h>

// messages
#include <excavaROB_mechanism_controllers/BaseControllerState.h>


using namespace std;

class BaseControllerStateRecord
{
  public:

    BaseControllerStateRecord();

    ~BaseControllerStateRecord();

    bool initialize();


  private:

    void baseControllerStateCallback(const excavaROB_mechanism_controllers::BaseControllerStateConstPtr& msg);

    void writeToDisc();



    ros::NodeHandle nh_, nh_private_;

    std::string path_name_, motion_name_;

    ros::Subscriber ctrl_state_sub_;

    excavaROB_mechanism_controllers::BaseControllerState ctrl_state_msg_;

    std::vector<std::vector<double> > vel_desired_, vel_actual_, u_;

    std::vector<double> t_;

    double t_start_;

    bool initialized_;

}; // @class BaseControllerStateRecord



BaseControllerStateRecord::BaseControllerStateRecord(): nh_private_("~")
{
  initialized_ = false;
}


BaseControllerStateRecord::~BaseControllerStateRecord()
{
  ctrl_state_sub_.shutdown();
  writeToDisc();
}


void BaseControllerStateRecord::writeToDisc()
{
  path_name_.append(motion_name_ + std::string(".txt"));

  std::ofstream outfile;
  outfile.open(path_name_.c_str());
  outfile.precision(4);
  outfile.setf(ios::fixed, ios::floatfield);
  outfile.setf(ios::left, ios::adjustfield);
  for (unsigned int j = 0; j < vel_desired_[0].size(); j++) {
    if (j == 0)
      outfile << "t" << '\t' << "v_d" << '\t' << "thd_d" << '\t' << "v_a" << '\t' << "thd_a" << '\t' << "ur" << '\t' << "ul" << endl;

    outfile << t_[j] << '\t';
    for (unsigned int i = 0; i < vel_desired_.size(); i++)
      outfile << vel_desired_[i][j] << '\t';

    for (unsigned int i = 0; i < vel_actual_.size(); i++)
      outfile << vel_actual_[i][j] << '\t';

    for (unsigned int i = 0; i < u_.size(); i++) {
      if (i == u_.size() - 1)
        outfile << u_[i][j] << endl;
      else
        outfile << u_[i][j] << '\t';
    }
  }
  outfile.close();
}


bool BaseControllerStateRecord::initialize()
{
  if (!nh_private_.getParam("path_name", path_name_)) {
    ROS_FATAL("Could not found path name from parameter server");
    return false;
  }
  if (!nh_private_.getParam("motion_name", motion_name_)) {
    ROS_FATAL("Could not found motion name from parameter server");
    return false;
  }

  ctrl_state_sub_ = nh_.subscribe<excavaROB_mechanism_controllers::BaseControllerState>("base_controller/state", 1, &BaseControllerStateRecord::baseControllerStateCallback, this);
  return true;
}

void BaseControllerStateRecord::baseControllerStateCallback(const excavaROB_mechanism_controllers::BaseControllerStateConstPtr& msg)
{
  ctrl_state_msg_ = *msg;
  if (!initialized_) {
    vel_desired_.resize(2);
    vel_actual_.resize(2);
    u_.resize(2);
    t_start_ = ros::Time::now().toSec();
    initialized_ = true;
  }

  t_.push_back(ros::Time::now().toSec() - t_start_);
  vel_desired_[0].push_back(ctrl_state_msg_.velocity_desired.linear);
  vel_desired_[1].push_back(ctrl_state_msg_.velocity_desired.angular);
  vel_actual_[0].push_back(ctrl_state_msg_.velocity_measured.linear);
  vel_actual_[1].push_back(ctrl_state_msg_.velocity_measured.angular);
  u_[0].push_back(ctrl_state_msg_.control_signal_right_wheel);
  u_[1].push_back(ctrl_state_msg_.control_signal_left_wheel);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_controller_state_record");

  BaseControllerStateRecord base_controller_state_record;

  if (!base_controller_state_record.initialize())
  {
    ROS_ERROR("Could not initialize base controller state record node.");
    return -1;
  }

  ros::spin();
  return 0;
}
