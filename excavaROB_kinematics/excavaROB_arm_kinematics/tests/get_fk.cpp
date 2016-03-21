#include <ros/ros.h>
#include <excavaROB_arm_kinematics_msgs/GetKinematicSolverInfo.h>
#include <excavaROB_arm_kinematics_msgs/GetPositionFK.h>

int main(int argc, char **argv){
    ros::init (argc, argv, "get_fk");
    ros::NodeHandle rh;

    ros::service::waitForService("excavaROB_arm_kinematics/get_kin_solver_info");
    ros::service::waitForService("excavaROB_arm_kinematics/arm_fk");

    ros::ServiceClient fk_info_client = rh.serviceClient<excavaROB_arm_kinematics_msgs::GetKinematicSolverInfo>("excavaROB_arm_kinematics/get_kin_solver_info");
    ros::ServiceClient fk_client = rh.serviceClient<excavaROB_arm_kinematics_msgs::GetPositionFK>("excavaROB_arm_kinematics/arm_fk");

    // define the service messages
    excavaROB_arm_kinematics_msgs::GetKinematicSolverInfo::Request info_request;
    excavaROB_arm_kinematics_msgs::GetKinematicSolverInfo::Response info_response;

    if (fk_info_client.call(info_request, info_response)) {
	for (unsigned int i = 0; i < info_response.kinematic_solver_info.joint_names.size(); i++) {
	    ROS_DEBUG("Joint: %d %s", i, info_response.kinematic_solver_info.joint_names[i].c_str());
        }
    }
    else {
	ROS_ERROR("Could not call query service");
	ros::shutdown();
	exit(1);
    }

    // define the service messages
    excavaROB_arm_kinematics_msgs::GetPositionFK::Request fk_request;
    excavaROB_arm_kinematics_msgs::GetPositionFK::Response fk_response;

    fk_request.header.frame_id = "drivetrain_link";
    fk_request.fk_link_names.resize(4);
    fk_request.fk_link_names[0] = "turret_link";
    fk_request.fk_link_names[1] = "boom_link";
    fk_request.fk_link_names[2] = "stick_link";
    fk_request.fk_link_names[3] = "bucket_link";

    fk_request.joint_state.position.resize(info_response.kinematic_solver_info.joint_names.size());
    fk_request.joint_state.name = info_response.kinematic_solver_info.joint_names;

    for (unsigned int i = 0; i < info_response.kinematic_solver_info.joint_names.size(); i++) {
	fk_request.joint_state.position[i] = 0.0;
    }
    if (fk_client.call(fk_request, fk_response)) {
	if (fk_response.error_code.val == fk_response.error_code.SUCCESS) {
	    for (unsigned int i=0; i < fk_response.pose_stamped.size(); i ++) {
	        ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[i].c_str());
                ROS_INFO_STREAM("Position: " << fk_response.pose_stamped[i].pose.position.x << "," 
					     << fk_response.pose_stamped[i].pose.position.y << ","
					     <<	fk_response.pose_stamped[i].pose.position.z);
                ROS_INFO("Orientation: %f %f %f %f",  fk_response.pose_stamped[i].pose.orientation.x,
                  				      fk_response.pose_stamped[i].pose.orientation.y,
                  				      fk_response.pose_stamped[i].pose.orientation.z,
                  				      fk_response.pose_stamped[i].pose.orientation.w); 
	    } 
	}
	else {
	    ROS_ERROR("Forward kinematics has failed");
	}
    }
    else {
	ROS_ERROR("Forward kinematics service call has failed");
    }
    ros::shutdown();
}

