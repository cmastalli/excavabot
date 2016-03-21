#include <ros/ros.h>
#include <ros/node_handle.h>

#include <fstream>

// kdl includes
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

// tf includes
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>

// messages
#include <sensor_msgs/JointState.h>
#include <motion_record/MotionRecord.h>


using namespace std;


class MotionRecord
{
  public:
    MotionRecord();

    ~MotionRecord();

    bool initialize();


  private:
    bool loadModel(const std::string xml);

    bool readJoints(urdf::Model &robot_model, const std::string root_link,
                    const std::string tip_link, unsigned int *num_joints);

    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

    bool getForwardKinematics();

    int getJointIndex(int index, const sensor_msgs::JointState &msg);

    bool storeMotionData();

    void writeToDisc();



    ros::NodeHandle nh_, nh_private_;

    std::string path_name_, motion_name_;

    std::string root_link_, wrist_link_;

    unsigned int num_joints_wrist_;

    KDL::JntArray joint_min_, joint_max_;
    KDL::Chain wrist_chain_;
    KDL::ChainFkSolverPos_recursive* fk_solver_;

    ros::Subscriber jnt_state_sub_;

    sensor_msgs::JointState jnt_state_msg_;

    std::vector<std::string> joint_names_;

    motion_record::MotionRecord motion_msg_;

    std::vector<double> t_;

    std::vector<double> x_wrist_, y_wrist_, z_wrist_, pitch_wrist_;

    std::vector<double> xd_wrist_, yd_wrist_, zd_wrist_, pitchd_wrist_;

    std::vector<double> xdd_wrist_, ydd_wrist_, zdd_wrist_, pitchdd_wrist_;

    geometry_msgs::PoseStamped pose_current_, pose_last_;

    double pitch_current_, pitch_last_, t0_;

    bool initialized_;

    ros::Time current_time_, last_time_;

}; // @class MotionRecord



MotionRecord::MotionRecord(): nh_private_("~")
{
  initialized_ = false;
}


MotionRecord::~MotionRecord()
{
  jnt_state_sub_.shutdown();
  writeToDisc();
}

void MotionRecord::writeToDisc()
{
  path_name_.append(motion_name_ + std::string(".txt"));

  std::ofstream outfile;
  outfile.open(path_name_.c_str());
  outfile.precision(4);
  outfile.setf(ios::fixed, ios::floatfield);
  outfile.setf(ios::left, ios::adjustfield);
//  outfile.setf(ios_base::skipws);
  for (unsigned int i = 0; i < x_wrist_.size(); i++)
  {
    if (i == 0)
    {
      outfile << "t" << '\t';
      outfile << "x" << '\t' << "y" << '\t' << "z" << '\t' << "p" << '\t';
      outfile << "xd"  << '\t' << "yd"  << '\t' << "zd"  << '\t' << "pd"  << '\t';
      outfile << "xdd" << '\t' << "ydd" << '\t' << "zdd" << '\t' << "pdd" << endl;
    }
    // time data
    outfile << t_[i] << '\t';
    // position data
    outfile << x_wrist_[i] << '\t';
    outfile << y_wrist_[i] << '\t';
    outfile << z_wrist_[i] << '\t';
    outfile << pitch_wrist_[i] << '\t';
    // velocity data
    outfile << xd_wrist_[i] << '\t';
    outfile << yd_wrist_[i] << '\t';
    outfile << zd_wrist_[i] << '\t';
    outfile << pitchd_wrist_[i] << '\t';
    // acceleration data
    outfile << xdd_wrist_[i] << '\t';
    outfile << ydd_wrist_[i] << '\t';
    outfile << zdd_wrist_[i] << '\t';
    outfile << pitchdd_wrist_[i] << endl;
  }
  outfile.close();
}


bool MotionRecord::initialize()
{
  std::string urdf_xml, full_urdf_xml;
  nh_.param("urdf_xml", urdf_xml, std::string("robot_description"));
  nh_.searchParam(urdf_xml, full_urdf_xml);
  ROS_DEBUG("Reading xml file from parameter server");

  std::string urdf_result;
  if (!nh_.getParam(full_urdf_xml, urdf_result))
  {
    ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }

  // Get root and wrist link name from parameter server
  if (!nh_private_.getParam("root_link", root_link_))
  {
    ROS_FATAL("Could not found root link from parameter server");
    return false;
  }
  if (!nh_private_.getParam("wrist_link", wrist_link_))
  {
    ROS_FATAL("Could not found wrist link from parameter server");
    return false;
  }
  if (!nh_private_.getParam("path_name", path_name_))
  {
    ROS_FATAL("Could not found motion name from parameter server");
    return false;
  }
  if (!nh_private_.getParam("motion_name", motion_name_))
  {
    ROS_FATAL("Could not found motion name from parameter server");
    return false;
  }

  // Load and read model
  if (!loadModel(urdf_result))
  {
    ROS_FATAL("Could not load models!");
    return false;
  }

  // Build forward kinematic solver
  fk_solver_ =  new KDL::ChainFkSolverPos_recursive(wrist_chain_);

  jnt_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &MotionRecord::jointStateCallback, this);

  return true;
}



bool MotionRecord::loadModel(const std::string xml)
{
  urdf::Model robot_model;
  KDL::Tree tree;

  if (!robot_model.initString(xml))
  {
    ROS_FATAL("Could not initialize robot model");
    return -1;
  }
  if (!kdl_parser::treeFromString(xml, tree))
  {
    ROS_FATAL("Could not initialize tree object");
    return false;
  }
  if (!tree.getChain(root_link_, wrist_link_, wrist_chain_))
  {
    ROS_FATAL("Could not initialize chain object");
    return false;
  }
  if (!readJoints(robot_model, root_link_, wrist_link_, &num_joints_wrist_))
  {
    ROS_FATAL("Could not read information about joints");
    return false;
  }

  return true;
}


bool MotionRecord::readJoints(urdf::Model &robot_model, 
                                   const std::string root_link,
                                   const std::string tip_link,
                                   unsigned int* num_joints)
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
    link =  robot_model.getLink(link->getParent()->name);
  }
  *num_joints = _num_joints;

  // Define arrays
  joint_min_.resize(_num_joints);
  joint_max_.resize(_num_joints);
  joint_names_.resize(_num_joints);

  // Get bounds for joints
  link = robot_model.getLink(tip_link);
  unsigned int i = 0;
  while (link && i < _num_joints)
  {
    joint = robot_model.getJoint(link->parent_joint->name);
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      ROS_INFO("getting bounds for joint: [%s]", joint->name.c_str());

//      bool hasLimits;
      float min, max;
      if (joint->type != urdf::Joint::CONTINUOUS)
      {
	min = joint->limits->lower;
	max = joint->limits->upper;
//	hasLimits = true;
      } 
      else
      {
	min = -M_PI;
	max = M_PI;
//	hasLimits = false;
      }

      int index = _num_joints - i - 1;
      joint_min_.data[index] = min;
      joint_max_.data[index] = max;
      joint_names_[index] = joint->name;

      i++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }

  return true;
}


void MotionRecord::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  jnt_state_msg_ = *msg;

  if (!this->getForwardKinematics())
    ROS_ERROR("Could not get forward kinematics");
}


bool MotionRecord::getForwardKinematics()
{
  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in;
  tf::Stamped<tf::Pose> tf_pose;


  // Get joints positions for root to wrist link
  int index;
  jnt_pos_in.resize(num_joints_wrist_);
  for (unsigned int i = 0; i < num_joints_wrist_; i++)
  {
    index = getJointIndex(i, jnt_state_msg_);
    if (index >= 0)
      jnt_pos_in(i) = jnt_state_msg_.position[index];
  }

//  int wrist_index = index+1;
  if (fk_solver_->JntToCart(jnt_pos_in, p_out, num_joints_wrist_) >= 0)
  {
    current_time_ = ros::Time::now();

    tf_pose.frame_id_ = root_link_;
    tf_pose.stamp_ = ros::Time();
    tf::PoseKDLToTF(p_out, tf_pose);

    tf::poseStampedTFToMsg(tf_pose, pose_current_);
    pitch_current_ = jnt_pos_in(num_joints_wrist_ - 1);//jnt_state_msg_.position[wrist_index];
  }

  // write to disc motion recorded message
  if(!storeMotionData())
    ROS_INFO("Could not store motion trajectory.");

  // set the last values of position and time
  pose_last_ = pose_current_;
  pitch_last_ = pitch_current_;
  last_time_ = current_time_;

  return true;
}


int MotionRecord::getJointIndex(int index, const sensor_msgs::JointState &msg)
{
  for (unsigned int i = 0; i < msg.name.size(); i++)
  {
    ROS_DEBUG("name: %s %i", msg.name[i].c_str(), (int) msg.name.size());
    ROS_DEBUG("joint name: %s", joint_names_[index].c_str());
    if (joint_names_[index] == msg.name[i])
      return i;
  }

  return -1;
}


bool MotionRecord::storeMotionData()
{
  if (!initialized_)
  {
    t0_ = current_time_.toSec();
    // pose messages
    pose_last_ = pose_current_;
    pitch_last_ = pitch_current_;
    // velocity messages
    motion_msg_.wrist_vel.x = 0.0;
    motion_msg_.wrist_vel.y = 0.0;
    motion_msg_.wrist_vel.z = 0.0;
    motion_msg_.wrist_angle_vel.pitch = 0.0;
    // acceleration messages
    motion_msg_.wrist_acc.x = 0.0;
    motion_msg_.wrist_acc.y = 0.0;
    motion_msg_.wrist_acc.z = 0.0;
    motion_msg_.wrist_angle_acc.pitch = 0.0;

    initialized_ = true;
  }
  else
  {
    double delta_x = pose_current_.pose.position.x - pose_last_.pose.position.x;
    double delta_y = pose_current_.pose.position.y - pose_last_.pose.position.y;
    double delta_z = pose_current_.pose.position.z - pose_last_.pose.position.z;
    double delta_pitch = pitch_current_ - pitch_last_;
    double dt = (current_time_ - last_time_).toSec();

    if (dt == 0.0)
      return false;

    // set velocity data
    motion_msg_.wrist_vel.x = delta_x / dt;
    motion_msg_.wrist_vel.y = delta_y / dt;
    motion_msg_.wrist_vel.z = delta_z / dt;
    motion_msg_.wrist_angle_vel.pitch = delta_pitch / dt;
    // set acceleration data
    motion_msg_.wrist_acc.x = delta_x / pow(dt,2);
    motion_msg_.wrist_acc.y = delta_y / pow(dt,2);
    motion_msg_.wrist_acc.z = delta_z / pow(dt,2);
    motion_msg_.wrist_angle_acc.pitch = delta_pitch / pow(dt,2);
  }

  // set motion messages
  motion_msg_.wrist_pos.x = pose_current_.pose.position.x;
  motion_msg_.wrist_pos.y = pose_current_.pose.position.y;
  motion_msg_.wrist_pos.z = pose_current_.pose.position.z;
  motion_msg_.wrist_angle_pos.pitch = pitch_current_;

  // set time data
  t_.push_back(current_time_.toSec() - t0_);
  // set position data
  x_wrist_.push_back(motion_msg_.wrist_pos.x);
  y_wrist_.push_back(motion_msg_.wrist_pos.y);
  z_wrist_.push_back(motion_msg_.wrist_pos.z);
  pitch_wrist_.push_back(motion_msg_.wrist_angle_pos.pitch);
  // set velocity data
  xd_wrist_.push_back(motion_msg_.wrist_vel.x);
  yd_wrist_.push_back(motion_msg_.wrist_vel.y);
  zd_wrist_.push_back(motion_msg_.wrist_vel.z);
  pitchd_wrist_.push_back(motion_msg_.wrist_angle_vel.pitch);
  // set acceleration data
  xdd_wrist_.push_back(motion_msg_.wrist_acc.x);
  ydd_wrist_.push_back(motion_msg_.wrist_acc.y);
  zdd_wrist_.push_back(motion_msg_.wrist_acc.z);
  pitchdd_wrist_.push_back(motion_msg_.wrist_angle_acc.pitch);

  ROS_DEBUG("Cartesian position: xyz = [%f %f %f]; pitch = %f at t = %f", motion_msg_.wrist_pos.x, motion_msg_.wrist_pos.y, motion_msg_.wrist_pos.z, motion_msg_.wrist_angle_pos.pitch, current_time_.toSec() - t0_);

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_record");

  MotionRecord motion_record;

  if (!motion_record.initialize())
  {
    ROS_ERROR("Could not initialize motion record node");
    return -1;
  }

  ros::spin();
  return 0;
}
