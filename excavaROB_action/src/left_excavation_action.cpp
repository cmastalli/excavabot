#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

class LeftExcavationAction
{
  private:
    TrajClient* traj_client_;

  public:
    LeftExcavationAction()
    {
      traj_client_ = new TrajClient("arm_controller/joint_trajectory_action", true);

      while(!traj_client_->waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the joint_trajectory_action server.");
      }
    }

    ~LeftExcavationAction()
    {
      delete traj_client_;
    }

    // sends the command to start a given trajectory
    void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
    {
      // when to start the trajectory: 1s from now
      goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
      traj_client_->sendGoal(goal);
    }

    pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
    {
      pr2_controllers_msgs::JointTrajectoryGoal goal;

      goal.trajectory.joint_names.push_back("turret_drivetrain_joint");
      goal.trajectory.joint_names.push_back("boom_turret_joint");
      goal.trajectory.joint_names.push_back("stick_boom_joint");
      goal.trajectory.joint_names.push_back("bucket_stick_joint");
      goal.trajectory.points.resize(4);

      // First trajectory point
      int ind = 0;
      goal.trajectory.points[ind].positions.resize(4);
      goal.trajectory.points[ind].positions[0] = 0.30;
      goal.trajectory.points[ind].positions[1] = -0.15;
      goal.trajectory.points[ind].positions[2] = 0.50;
      goal.trajectory.points[ind].positions[3] = 1.60;

      goal.trajectory.points[ind].velocities.resize(4);
      for (size_t j = 0; j < 4; ++j) {
        goal.trajectory.points[ind].velocities[j] = 0.0;
      }
      goal.trajectory.points[ind].time_from_start = ros::Duration(4);

      // Second trajectory point
      ind += 1;
      goal.trajectory.points[ind].positions.resize(4);
      goal.trajectory.points[ind].positions[0] = 0.30;
      goal.trajectory.points[ind].positions[1] = -0.15;
      goal.trajectory.points[ind].positions[2] = 0.50;
      goal.trajectory.points[ind].positions[3] = 1.50;

      goal.trajectory.points[ind].velocities.resize(4);
      for (size_t j = 0; j < 4; ++j) {
        goal.trajectory.points[ind].velocities[j] = 0.0;
      }
      goal.trajectory.points[ind].time_from_start = ros::Duration(2);

      // Third trajectory point
      ind += 1;
      goal.trajectory.points[ind].positions.resize(4);
      goal.trajectory.points[ind].positions[0] = 0.30;
      goal.trajectory.points[ind].positions[1] = -0.15;
      goal.trajectory.points[ind].positions[2] = -0.25;
      goal.trajectory.points[ind].positions[3] = -0.28;

      goal.trajectory.points[ind].velocities.resize(4);
      for (size_t j = 0; j < 4; ++j) {
        goal.trajectory.points[ind].velocities[j] = 0.0;
      }
      goal.trajectory.points[ind].time_from_start = ros::Duration(7);

      // Fourth trajectory point
      ind += 1;
      goal.trajectory.points[ind].positions.resize(4);
      goal.trajectory.points[ind].positions[0] = 0.00;
      goal.trajectory.points[ind].positions[1] = 0.30;
      goal.trajectory.points[ind].positions[2] = -0.55;
      goal.trajectory.points[ind].positions[3] = -0.28;

      goal.trajectory.points[ind].velocities.resize(4);
      for (size_t j = 0; j < 4; ++j) {
        goal.trajectory.points[ind].velocities[j] = 0.0;
      }
      goal.trajectory.points[ind].time_from_start = ros::Duration(3);


      return goal;
    }

    // returns the current state of the action
    actionlib::SimpleClientGoalState getState()
    {
      return traj_client_->getState();
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_driver");

  LeftExcavationAction arm;
  arm.startTrajectory(arm.armExtensionTrajectory());

  while(!arm.getState().isDone() && ros::ok()) {
    usleep(5000);
  }
}
