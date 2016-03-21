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


#include <tf/tf.h>

#include <excavaROB_mechanism_controllers/base_kinematics.h>

//#include <excavaROB_mechanism_controller/OdometryMatrix.h>
#include <excavaROB_mechanism_controllers/DebugInfo.h>
#include <excavaROB_mechanism_controllers/BaseOdometryState.h>
#include <excavaROB_mechanism_controllers/Odometer.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <realtime_tools/realtime_publisher.h>


namespace controller
{

  /*! \class
  \brief This class inherits from Controller and computes the base odometry
  */
  class ExcavaROBOdometry : public pr2_controller_interface::Controller
  {
   public:

    /*!
    * \brief Constructor for the odometry
    */
    ExcavaROBOdometry();

    /*!
    * \brief Destructor for the odometry
    */
    ~ExcavaROBOdometry();

    /*!
    * \brief Initializes and loads odometry information from the param server
    * @param robot_state The robot's current state
    * @param config Tiny xml element pointing to this controller
    * @return Successful init
    */
    bool init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node);

    /*
    * \brief  The starting method is called by the realtime thread just before
    * the first call to update.  Overrides Controller.staring().
    * @return Successful start
    */
    void starting();

    /*!
    * \brief The update method is called by the realtime thread after the call of starting. This method updates commands to wheels and it's called wvery timestep in realtime
    */
    void update();

    /*!
    * \brief Finds and stores the latest odometry information
    */
    void updateOdometry();

    /*!
    * \brief Publishes the currently computed odometry information
    */
    void publishOdometry();

    /*!
    * \brief Publishes the currently computed odometer information
    */
    void publishOdometer();

    /*!
    * \brief Publishes the odometry state information
    */
    void publishStateOdometry();

    /*!
    * \brief Publishes the currently computed odometry information to tf
    */
    void publishTransform();

    /*!
    * \brief Point that stores the current translational position (x,y) and angular position (z)
    */
    geometry_msgs::Point odom_;

    /*!
    * \brief Twist that remembers the current translational velocities (vel.vx, vel.vy) and angular position (ang_vel.vz)
    */
    geometry_msgs::Twist odom_vel_;






   private:

    ros::NodeHandle node_;

    /*!
    * \brief class where the robot's information is computed and stored
    */
    BaseKinematics base_kin_;

    /*!
    * \brief Check the inputs of velocity
    */
    bool isInputValid();
// UPDATEODOMETRY
    /*!
    * \brief Computes the base velocity from the wheel speeds
    */
    void computeBaseVelocity();

    /*!
    * \brief Builds the odometry message and prepares it for sending
    * @param msg The nav_msgs::Odometry into which the odometry values are placed
    */
    void getOdometryMessage(nav_msgs::Odometry &msg);

    /*!
    * \brief Takes the current odometery information and stores it into the six double parameters
    * @param x X component of the current odom position
    * @param y Y component of the current odom position
    * @param yaw Yaw (theta) component of the current odom position
    * @param vx X component of the current odom velocity
    * @param vy Y component of the current odom velocity
    * @param vw Angular velocity (omega) component of the current odom velocity
    */
    void getOdometry(double &x, double &y, double &yaw, double &vx, double &vy, double &vw);


    int sequence_;

    std::string tf_prefix_;

    /*!
    * \brief Total distance traveled by the base as computed by the odometer
    */
    double odometer_initial_distance_;

    /*!
    * \brief Total angular distance traveled by the base as computed by the odometer
    */
    double odometer_initial_angle_;

    /*!
    * \brief Enable or disable tf publishing
    */
    bool publish_tf_;

    /*!
    * \brief The topic name of the published odometry
    */
    std::string odom_frame_;

    /*!
    * \brief The topic name of the base footprint frame
    */
    std::string base_footprint_frame_;

    /*!
    * \brief The topic name of the base link frame
    */
    std::string base_link_frame_;

    /*!
    * \brief Distance between the two tracks
    */
    double track_distance_;

    double sigma_x_,sigma_y_,sigma_theta_,cov_xy_,cov_x_theta_,cov_y_theta_;

    bool verbose_, publish_odom_, publish_odometer_, publish_state_odometry_;

    double odom_publish_rate_, odometer_publish_rate_, odometry_state_publish_rate_;

    /*!
    * \brief The time that the odometry is expected to be published next
    */
    double expected_publish_time_;

    /*!
    * \brief The time that the odometry is expected to be published next
    */
    double expected_odometer_publish_time_;

    /*!
    * \brief The time that the odometry is expected to be published next
    */
    double expected_state_publish_time_;

    /*!
    * \brief The RealtimePublisher that does the realtime publishing of the debug info
    */
    boost::scoped_ptr<realtime_tools::RealtimePublisher<excavaROB_mechanism_controllers::DebugInfo> > debug_publisher_;

    /*!
    * \brief The RealtimePublisher that does the realtime publishing of the odometry state
    */
    boost::scoped_ptr<realtime_tools::RealtimePublisher<excavaROB_mechanism_controllers::BaseOdometryState> > odometry_state_publisher_;

    /*!
    * \brief The RealtimePublisher that does the realtime publishing of the odometry
    */
    boost::scoped_ptr<realtime_tools::RealtimePublisher<excavaROB_mechanism_controllers::Odometer> > odometer_publisher_;

   /*!
   * \brief The RealtimePublisher that does the realtiem publishing of the odometry estimation
   */
   boost::scoped_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odometry_publisher_;

    /*!
    * \brief Publishes the transform between the odometry frame and the base frame
    */
    boost::scoped_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > transform_publisher_ ;

    /*!
    * \brief Stores the last update time and the current update time
    */
    ros::Time last_time_, current_time_;

    /*!
    * \brief The last time the odometry information was published
    */
    ros::Time last_publish_time_;

    /*!
    * \brief The last time the odometry information was published
    */
    ros::Time last_transform_publish_time_;

    /*!
    * \brief The last time the odometry information was published
    */
    ros::Time last_state_publish_time_;

    /*!
    * \brief The last time the odometry information was published
    */
    ros::Time last_odometer_publish_time_;

    /*!
    * \brief The current time the rotation of the wheels was published
    */
    double current_r_wheel_rotation_, current_l_wheel_rotation_;

    /*!
    * \brief The last time the rotation of the wheels was published
    */
    double last_r_wheel_rotation_, last_l_wheel_rotation_;

    /*!
    * \brief Radius of the right and left wheel
    */
    double r_radius_, l_radius_;

    /*!
    * \brief Delta odometry
    */
    double odom_delta_x_, odom_delta_y_, odom_delta_th_;

    /*!
    * \brief Total distance traveled by the base as computed by the odometer
    */
    double odometer_distance_;

    /*!
    * \brief Total angular distance traveled by the base as computed by the odometer
    */
    double odometer_angle_;

  }; // class ExcavaROBOdometry
}
