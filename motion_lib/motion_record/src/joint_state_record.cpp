#include <ros/ros.h>
#include <ros/node_handle.h>

#include <fstream>

// kdl includes
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

// tf includes
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>

// messages
#include <sensor_msgs/JointState.h>


using namespace std;

class JointStateRecord
{
  public:
    JointStateRecord();

    ~JointStateRecord();

    bool initialize();


  private:
    bool loadModel(const std::string xml);

    bool readJoints(urdf::Model &robot_model, const std::string root_link, const std::string tip_link, unsigned int *num_joints);

    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

    int getJointIndex(int index, const sensor_msgs::JointState &msg);

    void writeToDisc();



    ros::NodeHandle nh_, nh_private_;

    std::string path_name_, motion_name_;

    std::string root_link_, tip_link_;

    unsigned int num_joints_arm_;

    KDL::JntArray joint_min_, joint_max_;
    KDL::Chain arm_chain_;

    ros::Subscriber jnt_state_sub_;

    sensor_msgs::JointState jnt_state_msg_;

    KDL::JntArray jnt_pos_current_, jnt_vel_current_, jnt_acc_current_, jnt_tau_current_, jnt_vel_last_;

    std::vector<std::string> joint_names_;

    std::vector<std::vector<double> > q_;

    std::vector<std::vector<double> > qd_;

    std::vector<std::vector<double> > qdd_;

    std::vector<std::vector<double> > tau_;

    bool initialized_;

    ros::Time current_time_, last_time_;

}; // @class JointStateRecord



JointStateRecord::JointStateRecord(): nh_private_("~")
{
  initialized_ = false;
}


JointStateRecord::~JointStateRecord()
{
  jnt_state_sub_.shutdown();
  writeToDisc();
}


void JointStateRecord::writeToDisc()
{
  path_name_.append(motion_name_ + std::string(".txt"));

  std::ofstream outfile;
  outfile.open(path_name_.c_str());
  outfile.precision(4);
  outfile.setf(ios::fixed, ios::floatfield);
  outfile.setf(ios::left, ios::adjustfield);
//  outfile.setf(ios_base::skipws);
  for (unsigned int j = 0; j < q_[0].size(); j++) {
    if (j == 0)
    {
      outfile << "q1" << '\t' << "q2" << '\t' << "q3" << '\t' << "q4" << '\t';
      outfile << "q1d"  << '\t' << "q2d"  << '\t' << "q3d"  << '\t' << "q4d"  << '\t';
      outfile << "q1dd" << '\t' << "q2dd" << '\t' << "q3dd" << '\t' << "q4dd" << '\t';
      outfile << "tau1" << '\t' << "tau2" << '\t' << "tau2" << '\t' << "tau4" << endl;
    }
    for (unsigned int i = 0; i < q_.size(); i++) {
      outfile << q_[i][j] << '\t';
    }
    for (unsigned int i = 0; i < q_.size(); i++) {
      outfile << qd_[i][j] << '\t';
    }
    for (unsigned int i = 0; i < q_.size(); i++) {
      outfile << qdd_[i][j] << '\t';
    }
    for (unsigned int i = 0; i < q_.size(); i++) {
      if (i == q_.size() - 1)
        outfile << tau_[i][j] << endl;
      else
        outfile << tau_[i][j] << '\t';
    }
  }
  outfile.close();
}


bool JointStateRecord::initialize()
{
  std::string urdf_xml, full_urdf_xml;
  nh_.param("urdf_xml", urdf_xml, std::string("robot_description"));
  nh_.searchParam(urdf_xml, full_urdf_xml);
  ROS_DEBUG("Reading xml file from parameter server");

  std::string urdf_result;
  if (!nh_.getParam(full_urdf_xml, urdf_result)) {
    ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }

  // Get root and wrist link name from parameter server
  if (!nh_private_.getParam("root_link", root_link_)) {
    ROS_FATAL("Could not found root link from parameter server");
    return false;
  }
  if (!nh_private_.getParam("tip_link", tip_link_)) {
    ROS_FATAL("Could not found tip link from parameter server");
    return false;
  }
  if (!nh_private_.getParam("path_name", path_name_)) {
    ROS_FATAL("Could not found path name from parameter server");
    return false;
  }
  if (!nh_private_.getParam("motion_name", motion_name_)) {
    ROS_FATAL("Could not found motion name from parameter server");
    return false;
  }

  // Load and read model
  if (!loadModel(urdf_result)) {
    ROS_FATAL("Could not load models!");
    return false;
  }

  jnt_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &JointStateRecord::jointStateCallback, this);
  return true;
}


bool JointStateRecord::loadModel(const std::string xml)
{
  urdf::Model robot_model;
  KDL::Tree tree;

  if (!robot_model.initString(xml)) {
    ROS_FATAL("Could not initialize robot model");
    return -1;
  }
  if (!kdl_parser::treeFromString(xml, tree)) {
    ROS_FATAL("Could not initialize tree object");
    return false;
  }
  if (!tree.getChain(root_link_, tip_link_, arm_chain_)) {
    ROS_FATAL("Could not initialize chain object");
    return false;
  }
  if (!readJoints(robot_model, root_link_, tip_link_, &num_joints_arm_)) {
    ROS_FATAL("Could not read information about joints");
    return false;
  }

  jnt_pos_current_.resize(num_joints_arm_);
  jnt_vel_current_.resize(num_joints_arm_);
  jnt_acc_current_.resize(num_joints_arm_);
  jnt_tau_current_.resize(num_joints_arm_);
  jnt_vel_last_.resize(num_joints_arm_);
  q_.resize(num_joints_arm_);
  qd_.resize(num_joints_arm_);
  qdd_.resize(num_joints_arm_);
  tau_.resize(num_joints_arm_);
  return true;
}


bool JointStateRecord::readJoints(urdf::Model &robot_model, const std::string root_link, const std::string tip_link, unsigned int* num_joints)
{
  unsigned int _num_joints = 0;

  // Get joint maximun and minimun physical bounds for joints
  boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_link);
  boost::shared_ptr<const urdf::Joint> joint;

  while (link && link->name != root_link)
  {
    joint = robot_model.getJoint(link->parent_joint->name);
    if (!joint)
    {
      ROS_ERROR("Could not find joint: %s", joint->name.c_str());
      return false;
    }
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      ROS_INFO("adding joint [%s]", joint->name.c_str());
      _num_joints++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }
  *num_joints = _num_joints;

  // Define arrays
  joint_names_.resize(_num_joints);

  // Get bounds for joints
  link = robot_model.getLink(tip_link);
  unsigned int i = 0;
  while (link && i < _num_joints)
  {
    joint = robot_model.getJoint(link->parent_joint->name);
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      ROS_INFO("getting names for joint: [%s]", joint->name.c_str());

      int index = _num_joints - i - 1;
      joint_names_[index] = joint->name;
      i++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }

  return true;
}


void JointStateRecord::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  jnt_state_msg_ = *msg;
  current_time_ = ros::Time::now();

  for (unsigned int i = 0; i < num_joints_arm_; i++)
  {
    int index = getJointIndex(i, jnt_state_msg_);
    if (index >= 0)
      jnt_vel_current_(i) = jnt_state_msg_.velocity[index];

      q_[i].push_back(jnt_state_msg_.position[index]);
      qd_[i].push_back(jnt_state_msg_.velocity[index]);
      tau_[i].push_back(jnt_state_msg_.effort[index]);
  }

  if (!initialized_) {
    jnt_vel_last_ = jnt_vel_current_;
    last_time_ = current_time_;
    for (unsigned int i = 0; i < num_joints_arm_; i++)
      qdd_[i].push_back(0.0);

    initialized_ = true;
  }
  else
  {
    double dt = (current_time_ - last_time_).toSec();
    for (unsigned int i = 0; i < num_joints_arm_; i++) {
      double delta_vel = jnt_vel_current_(i) - jnt_vel_last_(i);
      // set acceleration data
      qdd_[i].push_back(delta_vel / dt);
    }
  }

  jnt_vel_last_ = jnt_vel_current_;
  last_time_ = current_time_;
}


int JointStateRecord::getJointIndex(int index, const sensor_msgs::JointState &msg)
{
  for (unsigned int i = 0; i < msg.name.size(); i++)
  {
    ROS_DEBUG("name: %s %i", msg.name[i].c_str(), msg.name.size());
    ROS_DEBUG("joint name: %s", joint_names_[index].c_str());
    if (joint_names_[index] == msg.name[i])
      return i;
  }

  return -1;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_state_record");

  JointStateRecord joint_state_record;

  if (!joint_state_record.initialize())
  {
    ROS_ERROR("Could not initialize joint state record node");
    return -1;
  }

  ros::spin();
  return 0;
}
