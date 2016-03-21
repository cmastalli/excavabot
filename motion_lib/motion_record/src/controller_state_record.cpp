#include <ros/ros.h>
#include <ros/node_handle.h>

#include <fstream>

// messages
#include <dmp_controller/DMPJointControllerState.h>


using namespace std;

class ControllerStateRecord
{
  public:

    ControllerStateRecord();

    ~ControllerStateRecord();

    bool initialize();


  private:

    void controllerStateCallback(const dmp_controller::DMPJointControllerStateConstPtr& msg);

    void writeToDisc();



    ros::NodeHandle nh_, nh_private_;

    std::string path_name_, motion_name_;

    ros::Subscriber ctrl_state_sub_;

    dmp_controller::DMPJointControllerState ctrl_state_msg_;

    std::vector<std::vector<double> > q_desired_;

    std::vector<std::vector<double> > q_actual_;

    std::vector<std::vector<double> > u_;

    bool initialized_;

}; // @class ControllerStateRecord



ControllerStateRecord::ControllerStateRecord(): nh_private_("~")
{
  initialized_ = false;
}


ControllerStateRecord::~ControllerStateRecord()
{
  ctrl_state_sub_.shutdown();
  writeToDisc();
}


void ControllerStateRecord::writeToDisc()
{
  path_name_.append(motion_name_ + std::string(".txt"));

  std::ofstream outfile;
  outfile.open(path_name_.c_str());
  outfile.precision(4);
  outfile.setf(ios::fixed, ios::floatfield);
  outfile.setf(ios::left, ios::adjustfield);
//  outfile.setf(ios_base::skipws);
  for (unsigned int j = 0; j < q_desired_[0].size(); j++) {
    if (j == 0)
    {
      outfile << "q1_d" << '\t' << "q2_d" << '\t' << "q3_d" << '\t' << "q4_d" << '\t';
      outfile << "q1_a" << '\t' << "q2_a" << '\t' << "q3_a" << '\t' << "q4_a" << '\t';
      outfile <<   "u1" << '\t' <<   "u2" << '\t' <<   "u2" << '\t' <<   "u4" << endl;
    }
    for (unsigned int i = 0; i < q_desired_.size(); i++)
      outfile << q_desired_[i][j] << '\t';

    for (unsigned int i = 0; i < q_desired_.size(); i++)
      outfile << q_actual_[i][j] << '\t';

    for (unsigned int i = 0; i < q_desired_.size(); i++) {
      if (i == q_desired_.size() - 1)
        outfile << u_[i][j] << endl;
      else
        outfile << u_[i][j] << '\t';
    }
  }
  outfile.close();
}


bool ControllerStateRecord::initialize()
{
  if (!nh_private_.getParam("path_name", path_name_)) {
    ROS_FATAL("Could not found path name from parameter server");
    return false;
  }
  if (!nh_private_.getParam("motion_name", motion_name_)) {
    ROS_FATAL("Could not found motion name from parameter server");
    return false;
  }

//  jnt_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &JointStateRecord::jointStateCallback, this);
  ctrl_state_sub_ = nh_.subscribe<dmp_controller::DMPJointControllerState>("dmp_arm_controller/state", 1, &ControllerStateRecord::controllerStateCallback, this);
  return true;
}

void ControllerStateRecord::controllerStateCallback(const dmp_controller::DMPJointControllerStateConstPtr& msg)
{
  ctrl_state_msg_ = *msg;
  if (!initialized_) {
    q_desired_.resize(ctrl_state_msg_.joint_names.size());
    q_actual_.resize(ctrl_state_msg_.joint_names.size());
    u_.resize(ctrl_state_msg_.joint_names.size());
    initialized_ = true;
  }

  for (unsigned int i = 0; i < ctrl_state_msg_.joint_names.size(); i++) {
    q_desired_[i].push_back(ctrl_state_msg_.q_desired[i]);
    q_actual_[i].push_back(ctrl_state_msg_.q_actual[i]);
    u_[i].push_back(ctrl_state_msg_.u[i]);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_state_record");

  ControllerStateRecord controller_state_record;

  if (!controller_state_record.initialize())
  {
    ROS_ERROR("Could not initialize joint state record node");
    return -1;
  }

  ros::spin();
  return 0;
}
