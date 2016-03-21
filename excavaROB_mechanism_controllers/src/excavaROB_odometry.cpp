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

#include "excavaROB_mechanism_controllers/excavaROB_odometry.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_DECLARE_CLASS(excavaROB_mechanism_controllers, ExcavaROBOdometry, controller::ExcavaROBOdometry, pr2_controller_interface::Controller)

namespace controller {


ExcavaROBOdometry::ExcavaROBOdometry()
{
  sequence_ = 0;
}

ExcavaROBOdometry::~ExcavaROBOdometry()
{
}



bool ExcavaROBOdometry::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node)
{
  node_ = node;

  std::string tf_prefix_param;
  node.searchParam("tf_prefix", tf_prefix_param);
  node.getParam(tf_prefix_param, tf_prefix_);

  node.param("odometer/initial_distance", odometer_initial_distance_, 0.0);
  node.param("odometer/initial_angle", odometer_initial_angle_, 0.0);
  node.param("odom/initial_x", odom_.x, 0.0);
  node.param("odom/initial_y", odom_.y, 0.0);
  node.param("odom/initial_yaw", odom_.z, 0.0);

  node.param("publish_tf", publish_tf_, true);
  node.param<std::string> ("odom_frame", odom_frame_, "odom");
  node.param<std::string> ("base_footprint_frame", base_footprint_frame_, "base_footprint");
  node.param<std::string> ("base_link_frame", base_link_frame_, "drivetrain_link");
  node.param<double> ("track_distance", track_distance_, 2.2542373);

  node.param<double> ("x_stddev", sigma_x_, 0.002);
  node.param<double> ("y_stddev", sigma_y_, 0.002);
  node.param<double> ("rotation_stddev", sigma_theta_, 0.017);

  node.param<double> ("cov_xy", cov_xy_, 0.0);
  node.param<double> ("cov_xrotation", cov_x_theta_, 0.0);
  node.param<double> ("cov_yrotation", cov_y_theta_, 0.0);
  node.param<bool> ("verbose", verbose_, false);

  node.param("odom_publish_rate", odom_publish_rate_, 30.0);
  node.param("odometer_publish_rate", odometer_publish_rate_, 1.0);
  node.param("odometry_state_publish_rate", odometry_state_publish_rate_, 1.0);

  if (odom_publish_rate_ <= 0.0) {
    expected_publish_time_ = 0.0;
    publish_odom_ = false;
  }
  else {
    expected_publish_time_ = 1.0 / odom_publish_rate_;
    publish_odom_ = true;
  }

  if(odometer_publish_rate_ <= 0.0) {
    expected_odometer_publish_time_ = 0.0;
    publish_odometer_ = false;
  }
  else {
    expected_odometer_publish_time_ = 1.0 / odometer_publish_rate_;
    publish_odometer_ = true;
  }

  if(odometry_state_publish_rate_ <= 0.0) {
    expected_state_publish_time_ = 0.0;
    publish_state_odometry_ = false;
  }
  else {
    expected_state_publish_time_ = 1.0 / odometry_state_publish_rate_;
    publish_state_odometry_ = true;
  }

  if (!base_kin_.init(robot_state, node_))
    return false;

  for (int i = 0; i < base_kin_.right_num_wheels_; i++) {
    if (!base_kin_.right_wheel_[i].joint_->calibrated_) {
      ROS_ERROR("Base odometry could not start because the wheels were not calibrated. Relaunch the odometry after you see the wheel calibration finish.");
	return false;
      }
  }
  for (int i = 0; i < base_kin_.left_num_wheels_; i++) {
    if (!base_kin_.left_wheel_[i].joint_->calibrated_) {
      ROS_ERROR("Base odometry could not start because the wheels were not calibrated. Relaunch the odometry after you see the wheel calibration finish.");
      return false;
    }
  }

  if (verbose_) {
    debug_publisher_.reset(new realtime_tools::RealtimePublisher<excavaROB_mechanism_controllers::DebugInfo> (node_,"debug", 1));
    debug_publisher_->msg_.timing.resize(3);
  }

  odometry_state_publisher_.reset(new realtime_tools::RealtimePublisher<excavaROB_mechanism_controllers::BaseOdometryState> (node_,"state", 1));
  odometer_publisher_.reset(new realtime_tools::RealtimePublisher<excavaROB_mechanism_controllers::Odometer> (node_,"odometer", 1));
  odometry_publisher_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry> (node_,odom_frame_, 1));
  transform_publisher_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage> (node_,"/tf", 1));
  transform_publisher_->msg_.transforms.resize(1);

  odometry_state_publisher_->msg_.right_wheel_link_names.resize(base_kin_.right_num_wheels_);
  odometry_state_publisher_->msg_.left_wheel_link_names.resize(base_kin_.left_num_wheels_);
  return true;
}


void ExcavaROBOdometry::starting()
{
  current_time_ = base_kin_.robot_state_->getTime();
  last_time_ = base_kin_.robot_state_->getTime();

  last_publish_time_ = base_kin_.robot_state_->getTime();
  last_transform_publish_time_ = base_kin_.robot_state_->getTime();
  last_state_publish_time_ = base_kin_.robot_state_->getTime();
  last_odometer_publish_time_ = base_kin_.robot_state_->getTime();

  current_r_wheel_rotation_ = base_kin_.right_wheel_[0].joint_->position_;
  current_l_wheel_rotation_ = base_kin_.left_wheel_[0].joint_->position_;
  last_r_wheel_rotation_ = base_kin_.right_wheel_[0].joint_->position_;
  last_l_wheel_rotation_ = base_kin_.left_wheel_[0].joint_->position_;

  r_radius_ = base_kin_.right_wheel_[0].wheel_radius_;
  l_radius_ = base_kin_.left_wheel_[0].wheel_radius_;
}



bool ExcavaROBOdometry::isInputValid()
{
  for (int i = 0; i < base_kin_.right_num_wheels_; i++)
    if (isnan(base_kin_.right_wheel_[i].joint_->velocity_) || isnan(base_kin_.right_wheel_[i].joint_->velocity_))
      return false;
  for (int i = 0; i < base_kin_.left_num_wheels_; i++)
    if (isnan(base_kin_.left_wheel_[i].joint_->velocity_) || isnan(base_kin_.left_wheel_[i].joint_->velocity_))
      return false;

  return true;
}

void ExcavaROBOdometry::updateOdometry()
{
  double dt = (current_time_ - last_time_).toSec();

  current_r_wheel_rotation_ = base_kin_.right_wheel_[0].joint_->position_;
  current_l_wheel_rotation_ = base_kin_.left_wheel_[0].joint_->position_;
    
  double r_wheel_rotation_delta = current_r_wheel_rotation_ - last_r_wheel_rotation_;
  double l_wheel_rotation_delta = current_l_wheel_rotation_ - last_l_wheel_rotation_;

  double delta_theta = (r_radius_ * r_wheel_rotation_delta - l_radius_ * l_wheel_rotation_delta) / track_distance_;
  double delta_s = (r_radius_ * r_wheel_rotation_delta + l_radius_ * l_wheel_rotation_delta) / 2;

  if (delta_theta / dt < 0.001) {
    odom_delta_x_ = delta_s * cos(odom_.z + delta_theta / 2);
    odom_delta_y_ = delta_s * sin(odom_.z + delta_theta / 2);
    odom_delta_th_ = delta_theta;
  }
  else {
    odom_delta_x_ = (delta_s / delta_theta) * (sin(odom_.z + delta_theta) - sin(odom_.z));
    odom_delta_y_ = -(delta_s / delta_theta) * (cos(odom_.z + delta_theta) - cos(odom_.z));
    odom_delta_th_ = delta_theta;
  }

  odom_.x += odom_delta_x_;
  odom_.y += odom_delta_y_;
  odom_.z += odom_delta_th_;

  odom_vel_.linear.x = odom_delta_x_ / dt;
  odom_vel_.linear.y = odom_delta_y_ / dt;
  odom_vel_.angular.z = odom_delta_th_ / dt;

  ROS_DEBUG("Odometry:: Position [x,y,theta]: %f, %f, %f", odom_.x, odom_.y, odom_.z);

  odometer_distance_ += sqrt(odom_delta_x_ * odom_delta_x_ + odom_delta_y_ * odom_delta_y_);
  odometer_angle_ += fabs(odom_delta_th_);
}


void ExcavaROBOdometry::update()
{
  if (!isInputValid()) {
    if (verbose_)
      debug_publisher_->msg_.input_valid = false;
      ROS_DEBUG("Odometry: Input velocities are invalid");
      return;
    }
  else {
    if (verbose_)
      debug_publisher_->msg_.input_valid = true;
  }

  current_time_ = base_kin_.robot_state_->getTime();
  ros::Time update_start = ros::Time::now();
  updateOdometry();

//    double update_time = (ros::Time::now()-update_start).toSec();
  ros::Time publish_start = ros::Time::now();
  if (publish_odom_)
    publishOdometry();
  if (publish_odometer_)
    publishOdometer();
  if (publish_state_odometry_)
    publishStateOdometry();
  if (publish_tf_)
    publishTransform();


  last_r_wheel_rotation_ = current_r_wheel_rotation_;
  last_l_wheel_rotation_ = current_l_wheel_rotation_;
  last_time_ = current_time_;
  sequence_ ++;
}


void ExcavaROBOdometry::getOdometryMessage(nav_msgs::Odometry &msg)
{
  msg.header.frame_id = odom_frame_;
  msg.header.stamp = current_time_;
  msg.pose.pose.position.x = odom_.x;
  msg.pose.pose.position.y = odom_.y;
  msg.pose.pose.position.z = 0.0;
    
  btQuaternion quat_trans;
  quat_trans.setEulerZYX(odom_.z, 0.0, 0.0);
  msg.pose.pose.orientation.x = quat_trans.x();
  msg.pose.pose.orientation.y = quat_trans.y();
  msg.pose.pose.orientation.z = quat_trans.z();
  msg.pose.pose.orientation.w = quat_trans.w();

  msg.twist.twist = odom_vel_;

  double  odom_multiplier = 1.0;

  if(fabs(odom_vel_.linear.x) <= 1e-8 && fabs(odom_vel_.linear.y) <= 1e-8 && fabs(odom_vel_.angular.z) <= 1e-8) {
  //nav_msgs::Odometry has a 6x6 covariance matrix
    msg.pose.covariance[0] = 1e-12;
    msg.pose.covariance[7] = 1e-12;
    msg.pose.covariance[35] = 1e-12;

    msg.pose.covariance[1] = 1e-12;
    msg.pose.covariance[6] = 1e-12;

    msg.pose.covariance[31] = 1e-12;
    msg.pose.covariance[11] = 1e-12;

    msg.pose.covariance[30] = 1e-12;
    msg.pose.covariance[5] =  1e-12;
  }
  else {
  //nav_msgs::Odometry has a 6x6 covariance matrix
    msg.pose.covariance[0] = odom_multiplier*pow(sigma_x_,2);
    msg.pose.covariance[7] = odom_multiplier*pow(sigma_y_,2);
    msg.pose.covariance[35] = odom_multiplier*pow(sigma_theta_,2);

    msg.pose.covariance[1] = odom_multiplier*cov_xy_;
    msg.pose.covariance[6] = odom_multiplier*cov_xy_;

    msg.pose.covariance[31] = odom_multiplier*cov_y_theta_;
    msg.pose.covariance[11] = odom_multiplier*cov_y_theta_;

    msg.pose.covariance[30] = odom_multiplier*cov_x_theta_;
    msg.pose.covariance[5] =  odom_multiplier*cov_x_theta_;
  }

  msg.pose.covariance[14] = DBL_MAX;
  msg.pose.covariance[21] = DBL_MAX;
  msg.pose.covariance[28] = DBL_MAX;

  msg.twist.covariance = msg.pose.covariance;
}


void ExcavaROBOdometry::publishOdometry()
{
  if (fabs((last_publish_time_ - current_time_).toSec()) < expected_publish_time_)
    return;

  if (odometry_publisher_->trylock()) {
    getOdometryMessage(odometry_publisher_->msg_);
    odometry_publisher_->unlockAndPublish();
    last_publish_time_ = current_time_;
  }
}


void ExcavaROBOdometry::publishOdometer()
{
  if (fabs((last_odometer_publish_time_ - current_time_).toSec()) < expected_odometer_publish_time_)
    return;

  if (odometer_publisher_->trylock()) {
    odometer_publisher_->msg_.distance = odometer_distance_;
    odometer_publisher_->msg_.angle = odometer_angle_;
    odometer_publisher_->unlockAndPublish();
    last_odometer_publish_time_ = current_time_;
  }
}


void ExcavaROBOdometry::publishStateOdometry()
{
  if (fabs((last_state_publish_time_ - current_time_).toSec()) < expected_state_publish_time_)
    return;

  if (odometry_state_publisher_->trylock()) {
    for (int i = 0; i < base_kin_.right_num_wheels_; i++) {
      odometry_state_publisher_->msg_.right_wheel_link_names[i] = base_kin_.right_wheel_[i].link_name_;
    }
    for (int i = 0; i < base_kin_.left_num_wheels_; i++) {
      odometry_state_publisher_->msg_.left_wheel_link_names[i] = base_kin_.left_wheel_[i].link_name_;
    }
    odometry_state_publisher_->msg_.velocity = odom_vel_;
    odometry_state_publisher_->unlockAndPublish();
    last_state_publish_time_ = current_time_;
  }
}

/*
  void ExcavaROBOdometry::getOdometry(geometry_msgs::Point &odom, geometry_msgs::Twist &odom_vel)
  {
    odom = odom_;
    odom_vel = odom_vel_;
    return;
  }
*/
void ExcavaROBOdometry::getOdometry(double &x, double &y, double &yaw, double &vx, double &vy, double &vyaw)
{
  x = odom_.x;
  y = odom_.y;
  yaw = odom_.z;
  vx = odom_vel_.linear.x;
  vy = odom_vel_.linear.y;
  vyaw = odom_vel_.angular.z;
}


void ExcavaROBOdometry::publishTransform()
{
  if (fabs((last_transform_publish_time_ - current_time_).toSec()) < expected_publish_time_)
    return;

  if (transform_publisher_->trylock()) {
    double x(0.0), y(0.0), yaw(0.0), vx(0.0), vy(0.0), vyaw(0.0);
    this->getOdometry(x, y, yaw, vx, vy, vyaw);

    geometry_msgs::TransformStamped &out = transform_publisher_->msg_.transforms[0];
    out.header.stamp = current_time_;
    out.header.frame_id = tf::resolve(tf_prefix_, base_footprint_frame_);
    out.child_frame_id = tf::resolve(tf_prefix_, odom_frame_);
    out.transform.translation.x = -x * cos(yaw) - y * sin(yaw);
    out.transform.translation.y =  x * sin(yaw) - y * cos(yaw);
    out.transform.translation.z = 0;

    btQuaternion quat_trans;
    quat_trans.setEulerZYX(-yaw, 0.0, 0.0);
    out.transform.rotation.x = quat_trans.x();
    out.transform.rotation.y = quat_trans.y();
    out.transform.rotation.z = quat_trans.z();
    out.transform.rotation.w = quat_trans.w();

    transform_publisher_->unlockAndPublish();
    last_transform_publish_time_ = current_time_;
  }
}

}  //@namespace controller


