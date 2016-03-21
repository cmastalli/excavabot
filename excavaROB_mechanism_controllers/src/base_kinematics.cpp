/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Simon Bolivar University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*
 * Author: Carlos Mastalli
 */

#include <excavaROB_mechanism_controllers/base_kinematics.h>

using namespace controller;


bool Wheel::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node, std::string link_name)
{
  wheel_stuck_ = 0;
  direction_multiplier_ = 1;
  wheel_speed_filtered_ = 0;
  wheel_speed_error_ = 0;
  wheel_speed_cmd_ = 0;
  wheel_speed_actual_ = 0;

  boost::shared_ptr<const urdf::Link> link;
  link = robot_state->model_->robot_model_.getLink(link_name);
  if(!link){
    ROS_ERROR("Could not find link with name %s", link_name.c_str());
    return false;
  }

  if(!link->collision->geometry)
  {
    ROS_ERROR("Link %s does not have collision geometry description. Add collision geometry description for link to urdf.", link_name.c_str());
    return false;
  }
  if(link->collision->geometry->type != urdf::Geometry::CYLINDER)
  {
    ROS_ERROR("Link %s does not have cylinder geometry", link_name.c_str());
    return false;
  }
  wheel_radius_ = (dynamic_cast<urdf::Cylinder*>(link->collision->geometry.get()))->radius;
  ROS_DEBUG("wheel name: %s, radius: %f", link_name.c_str(), wheel_radius_);
  link_name_ = link_name;
  joint_name_ = link->parent_joint->name;

  joint_ = robot_state->getJointState(joint_name_);
  if(!joint_){
    ROS_ERROR("Could not find joint with name %s", joint_name_.c_str());
    return false;
  }

  ROS_DEBUG("Loading wheel: %s", link_name_.c_str());
  return true;
}


bool BaseKinematics::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node)
{
  std::string right_wheel_names_string;
  std::string left_wheel_names_string;
  std::vector<std::string> right_wheel_names;
  std::vector<std::string> left_wheel_names;
  name_ = node.getNamespace();
  //Initialize stuff
  MAX_DT_ = 0.01;
  right_num_wheels_ = 0;
  left_num_wheels_ = 0;

  robot_state_ = robot_state;

  node.param<std::string> ("right_wheel_names", right_wheel_names_string, "");
  node.param<std::string> ("left_wheel_names", left_wheel_names_string, "");
  std::stringstream ss_r(right_wheel_names_string);
  std::stringstream ss_l(left_wheel_names_string);
  std::string tmp_r;
  std::string tmp_l;
  while(ss_r >> tmp_r)
  {
    right_wheel_names.push_back(tmp_r);
  }
  while(ss_l >> tmp_l)
  {
    left_wheel_names.push_back(tmp_l);
  }

  for(unsigned int i = 0; i < right_wheel_names.size(); i++)
  {
    Wheel tmp;
    right_wheel_.push_back(tmp);
    ROS_DEBUG("right wheel name: %s", right_wheel_names[i].c_str());
    ros::NodeHandle n(name_);
    if(!right_wheel_[right_num_wheels_].init(robot_state, n, right_wheel_names[i]))
    {
      ROS_ERROR("Could not initialize base kinematics");
      return false;
    }
    right_num_wheels_++;
  }
  for(unsigned int i = 0; i < left_wheel_names.size(); i++)
  {
    Wheel tmp;
    left_wheel_.push_back(tmp);
    ROS_DEBUG("left wheel name: %s", left_wheel_names[i].c_str());
    ros::NodeHandle n(name_);
    if(!left_wheel_[left_num_wheels_].init(robot_state, n, left_wheel_names[i]))
    {
      ROS_ERROR("Could not initialize base kinematics");
      return false;
    }
    left_num_wheels_++;
  }

  double multiplier;
  node.param<double> ("wheel_radius_multiplier", multiplier, 1.0);

  for(int i = 0; i < right_num_wheels_; i++)
  {
    right_wheel_[i].wheel_radius_ *= multiplier;
  }
  for(int i = 0; i < left_num_wheels_; i++)
  {
    left_wheel_[i].wheel_radius_ *= multiplier;
  }

  return true;
}










