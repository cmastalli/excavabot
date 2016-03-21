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

#ifndef EXCAVAROB_BASE_KINEMATICS_H
#define EXCAVAROB_BASE_KINEMATICS_H

#include <pr2_mechanism_model/robot.h>
#include <pr2_controller_interface/controller.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <boost/thread/condition.hpp>

namespace controller
{
  class Wheel;
  class BaseKinematics;

  /*! \class
   \brief This class keeps track of the wheels
   */
  class Wheel
  {
    public:
      /*!
       * \brief Loads wheel's information from the xml description file and param server
       * @param robot_state The robot's current state
       * @param config Tiny xml element pointing to this wheel
       */
      bool init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node, std::string link_name);


      /*!
       * \brief remembers if the wheel is stalled
       */
      int wheel_stuck_;

      /*!
       * \brief specifies the default direction of the wheel
       */
      int direction_multiplier_;

      /*!
       * \brief wheel speed filtered with alpha
       */
      double wheel_speed_filtered_;

      /*!
       * \brief difference between desired and actual speed
       */
      double wheel_speed_error_;

      /*!
       * \brief desired wheel speed
       */
      double wheel_speed_cmd_;

      /*!
       * \brief actual wheel speeds
       */
      double wheel_speed_actual_;

      /*!
       * \brief name of the link
       */
      std::string link_name_;

      /*!
       * \brief wheel radius scale (based on the default wheel radius in Basekinematics)
       */
      double wheel_radius_;

      /*!
       * \brief name of the joint
       */
      std::string joint_name_;

      /*!
       * \brief JointState for this wheel joint
       */
      pr2_mechanism_model::JointState *joint_;

      /*!
       * \brief BaseKinematics to which this wheel belongs
       */
      BaseKinematics *parent_;

  }; // class Wheel


  /*! \class
   \brief This class includes common functions used by the base controller and odometry
   */
  class BaseKinematics
  {
   public:

      /*!
       * \brief Loads BaseKinematic's information from the xml description file and param server
       * @param robot_state The robot's current state
       * @param config Tiny xml element pointing to its controller
       * @return Successful init
       */
   bool init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node);


      /*!
       * \brief remembers everything about the state of the robot
       */
      pr2_mechanism_model::RobotState *robot_state_;

      /*!
       * \brief name of this BaseKinematics (generally associated with whatever controller is using it)
       */
      std::string name_;

      /*!
       * \brief maximum dT used in computation of interpolated velocity command
       */
      double MAX_DT_;

      /*!
       * \brief number of wheels connected to the base
       */
      int right_num_wheels_, left_num_wheels_;

      /*!
       * \brief vector of every wheel attached to the base
       */
      std::vector<Wheel> right_wheel_, left_wheel_;

      /*!
       * \brief JointState for this wheel joint
       */
      pr2_mechanism_model::JointState *joint_;

  }; // class BaseKinematics

}  // namespace controller
#endif
