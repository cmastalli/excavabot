#include <ros/ros.h>
#include <excavaROB_arm_kinematics_msgs/GetKinematicSolverInfo.h>
#include <excavaROB_arm_kinematics_msgs/GetPositionIK.h>

int main(int argc, char **argv){
    ros::init (argc, argv, "get_ik");
    ros::NodeHandle rh;

    ros::service::waitForService("excavaROB_arm_kinematics/get_kin_solver_info");
    ros::service::waitForService("excavaROB_arm_kinematics/arm_ik");

    ros::ServiceClient info_client = rh.serviceClient<excavaROB_arm_kinematics_msgs::GetKinematicSolverInfo>("excavaROB_arm_kinematics/get_kin_solver_info");
    ros::ServiceClient ik_client = rh.serviceClient<excavaROB_arm_kinematics_msgs::GetPositionIK>("excavaROB_arm_kinematics/arm_ik");

    // define the service messages
    excavaROB_arm_kinematics_msgs::GetKinematicSolverInfo::Request info_request;
    excavaROB_arm_kinematics_msgs::GetKinematicSolverInfo::Response info_response;

    if (info_client.call(info_request, info_response)) {
	for (unsigned int i = 0; i < info_response.kinematic_solver_info.joint_names.size(); i++) {
	    ROS_INFO("Joint: %d %s", i, info_response.kinematic_solver_info.joint_names[i].c_str());//DEBUG
	}
    } else {
	ROS_ERROR("Could not call query service");
	ros::shutdown();
	exit(1);
    }

    // define the service messages
    excavaROB_arm_kinematics_msgs::GetPositionIK::Request  ik_request;
    excavaROB_arm_kinematics_msgs::GetPositionIK::Response ik_response;

    ik_request.header.frame_id = "drivetrain_link";
    ik_request.pose.position.x = 5.90701;//6.05617;//5.90701
    ik_request.pose.position.y = 2.13754;//4.19358;//2.13754
    ik_request.pose.position.z = 0.4005;//-2.6924;//0.40005
    ik_request.pose.pitch = 0.0;//-2.6924;//0.0
    ik_request.velocity.linear.x = 0.0;
    ik_request.velocity.linear.y = 0.0;
    ik_request.velocity.linear.z = 0.0;
    ik_request.velocity.angular.x = 0.0;
    ik_request.ik_timeout = ros::Duration(5.0);
    ik_request.ik_link_name = "stick_link";

    ik_request.ik_seed_state.position.resize(info_response.kinematic_solver_info.joint_names.size());
    ik_request.ik_seed_state.velocity.resize(info_response.kinematic_solver_info.joint_names.size());
    ik_request.ik_seed_state.name = info_response.kinematic_solver_info.joint_names;
    for (unsigned int i = 0; i < info_response.kinematic_solver_info.joint_names.size(); i++) {
	ik_request.ik_seed_state.position[i] = (info_response.kinematic_solver_info.limits[i].min_position + 
						info_response.kinematic_solver_info.limits[i].max_position) / 2.0;
	ik_request.ik_seed_state.velocity[i] = 0.0;
    }

    if (ik_client.call(ik_request, ik_response)) {
	if (ik_response.error_code.val == ik_response.error_code.SUCCESS) {
	    for (unsigned int i = 0; i < ik_response.ik_solution.name.size(); i++) {
		ROS_INFO("Joint: %s %f", ik_response.ik_solution.name[i].c_str(), ik_response.ik_solution.position[i]);
	    }
	} else{
	    ROS_ERROR("Inverse kinematics has failed");
	}
    } else
	ROS_ERROR("Inverse kinematics service call has failed");

    ros::shutdown();
}
