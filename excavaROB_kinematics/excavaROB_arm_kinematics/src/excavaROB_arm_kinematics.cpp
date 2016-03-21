// ROS includes
#include <cstring>
#include <ros/ros.h>

// KDL kinematics includes
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

// Kinematics messages includes
#include <excavaROB_arm_kinematics_msgs/GetPositionIK.h>
#include <excavaROB_arm_kinematics_msgs/GetPositionFK.h>
#include <excavaROB_arm_kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/KinematicSolverInfo.h>

// TF transformation includes
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>

// Eigen matrix include
#include <Eigen/Eigen>


USING_PART_OF_NAMESPACE_EIGEN
using std::string;

static const std::string IK_SERVICE = "arm_ik";
static const std::string FK_SERVICE = "arm_fk";
static const std::string KIN_INFO_SERVICE = "get_kin_solver_info";

class ExcavaROBArmKinematics {
    public:
        ExcavaROBArmKinematics();
	bool init();

    private:
        /**
         * @brief This is the basic IK service method that will compute and return an IK solution.
         * @param A request message. See service definition for GetPositionIK for more information on this message.
         * @param The response message. See service definition for GetPositionIK for more information on this message.
         */
        bool getPositionIK(excavaROB_arm_kinematics_msgs::GetPositionIK::Request &request,
                           excavaROB_arm_kinematics_msgs::GetPositionIK::Response &response);

        /**
         * @brief This is the basic forward kinematics service that will return information about the kinematics node.
         * @param A request message. See service definition for GetPositionFK for more information on this message.
         * @param The response message. See service definition for GetPositionFK for more information on this message.
         */
        bool getPositionFK(excavaROB_arm_kinematics_msgs::GetPositionFK::Request &request,
                           excavaROB_arm_kinematics_msgs::GetPositionFK::Response &response);

        /**
         * @brief This is the basic kinematics info service that will return information about the kinematics node.
         * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
         * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
         */
        bool getKinSolverInfo(excavaROB_arm_kinematics_msgs::GetKinematicSolverInfo::Request &request,
                              excavaROB_arm_kinematics_msgs::GetKinematicSolverInfo::Response &response);


	ros::NodeHandle nh_, nh_private_;
	std::string root_name_, wrist_name_, tip_name_;
	KDL::JntArray joint_min_, joint_max_;
	KDL::Chain arm_chain_;
	unsigned int num_joints_arm_;
	int max_iterations_;
	double eps_;
	double ik_gain_;
	double damping_;

	KDL::ChainFkSolverPos_recursive* fk_solver_;
        KDL::ChainJntToJacSolver* jnt_to_jac_solver_;
        KDL::ChainFkSolverPos* jnt_to_pose_solver_;

	ros::ServiceServer ik_service_, fk_service_;
	ros::ServiceServer kin_solver_info_service_;

        tf::TransformListener tf_listener_;

        bool loadModel(const std::string xml);
	bool readJoints(urdf::Model &robot_model, const std::string tip_name, const std::string root_name, unsigned int *_num_joints);
        int getJointIndex(const std::string &name);
        int getKDLSegmentIndex(const std::string &name);

        kinematics_msgs::KinematicSolverInfo info_;
};


ExcavaROBArmKinematics::ExcavaROBArmKinematics(): nh_private_ ("~") {
}

bool ExcavaROBArmKinematics::init() {
    // Get URDF XML
    std::string urdf_xml, full_urdf_xml;
    nh_.param("urdf_xml", urdf_xml, std::string("robot_description"));
    nh_.searchParam(urdf_xml, full_urdf_xml);
    ROS_DEBUG("ExcavaROB Kinematics: Reading xml file from parameter server");
    std::string result;
    if (!nh_.getParam(full_urdf_xml, result)) {
        ROS_FATAL("ExcavaROB Kinematics: Could not load the xml from parameter server: %s", urdf_xml.c_str());
	return false;
    }

    // Get Root, Wrist and Tip From Parameter Service
    if (!nh_private_.getParam("root_name", root_name_)) {
        ROS_FATAL("ExcavaROB Kinematics: No root_name found on parameter server");
        return false;
    }
    if (!nh_private_.getParam("tip_name", tip_name_)) {
        ROS_FATAL("ExcavaROB Kinematics: No tip_name found on parameter server");
        return false;
    }
    if (!nh_private_.getParam("max_iterations", max_iterations_)) {
        ROS_FATAL("ExcavaROB Kinematics: No max_iterations found on parameter server");
        return false;
    }
    if (!nh_private_.getParam("eps", eps_)) {
        ROS_FATAL("ExcavaROB Kinematics: No eps found on parameter server");
        return false;
    }
    if (!nh_private_.getParam("ik_gain", ik_gain_)) {
        ROS_FATAL("ExcavaROB Kinematics: No ik_gain found on parameter server");
        return false;
    }

    // Load and Read Models
    if (!loadModel(result)) {
        ROS_FATAL("Could not load models!");
        return false;
    }

    // Get Solver Parameters
    int maxIterations;
    double epsilon;
    nh_private_.param("maxIterations", maxIterations, 1000);
    nh_private_.param("epsilon", epsilon, 1e-2);

    // Build Solvers
    fk_solver_ = new KDL::ChainFkSolverPos_recursive(arm_chain_);
    jnt_to_jac_solver_ = new KDL::ChainJntToJacSolver(arm_chain_);
    jnt_to_pose_solver_ = new KDL::ChainFkSolverPos_recursive(arm_chain_);

    ROS_INFO("ExcavaROB Kinematics: Advertising services");
    ik_service_ = nh_private_.advertiseService(IK_SERVICE, &ExcavaROBArmKinematics::getPositionIK, this);
    fk_service_ = nh_private_.advertiseService(FK_SERVICE, &ExcavaROBArmKinematics::getPositionFK, this);

    kin_solver_info_service_ = nh_private_.advertiseService(KIN_INFO_SERVICE, &ExcavaROBArmKinematics::getKinSolverInfo, this);

    return true;
}

bool ExcavaROBArmKinematics::loadModel(const std::string xml) {
    urdf::Model robot_model;
    KDL::Tree tree;

    if (!robot_model.initString(xml)) {
	ROS_FATAL("Could not initialize robot model");
	return false;
    }
    if (!kdl_parser::treeFromString(xml, tree)) {
        ROS_ERROR("Could not initialize tree object");
        return false;
    }
    if (!tree.getChain(root_name_, tip_name_, arm_chain_)) {
        ROS_ERROR("Could not initialize chain object");
        return false;
    }
    if (!readJoints(robot_model, tip_name_, root_name_, &num_joints_arm_)) {
        ROS_FATAL("Could not read information about the joints");
        return false;
    }

    return true;
}

bool ExcavaROBArmKinematics::readJoints(urdf::Model &robot_model,
                                        const std::string tip_name,
                                        const std::string root_name,
                                        unsigned int *_num_joints) {
    unsigned int num_joints = 0;

    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint;

    while (link && link->name != root_name) {   
        ROS_DEBUG("Link: %s", link->name.c_str());    
        joint = robot_model.getJoint(link->parent_joint->name);
        if (!joint) {
            ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
            return false;
        }
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            ROS_DEBUG("Adding joint: [%s]", joint->name.c_str() );
            num_joints++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }
	*_num_joints = num_joints;

    joint_min_.resize(num_joints);
    joint_max_.resize(num_joints);
    info_.joint_names.resize(num_joints);
    info_.limits.resize(num_joints);

    link = robot_model.getLink(tip_name);
    unsigned int i = 0;
    while (link && i < num_joints) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            ROS_DEBUG( "Getting bounds for joint: [%s]", joint->name.c_str() );

            float lower, upper;
            int hasLimits;
            if (joint->type != urdf::Joint::CONTINUOUS) {
                lower = joint->limits->lower;
                upper = joint->limits->upper;
                hasLimits = 1;
            } else {
                lower = -M_PI;
                upper = M_PI;
                hasLimits = 0;
            }
            int index = num_joints - i -1;

            joint_min_.data[index] = lower;
            joint_max_.data[index] = upper;
            info_.joint_names[index] = joint->name;
            info_.limits[index].joint_name = joint->name;
            info_.limits[index].has_position_limits = hasLimits;
            info_.limits[index].min_position = lower;
            info_.limits[index].max_position = upper;
            i++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }
    return true;
}


int ExcavaROBArmKinematics::getJointIndex(const std::string &name) {
    for (unsigned int i = 0; i < info_.joint_names.size(); i++) {
        if (info_.joint_names[i] == name)
            return i;
    }
    return -1;
}

int ExcavaROBArmKinematics::getKDLSegmentIndex(const std::string &name) {
    int i=0; 
    while (i < (int) arm_chain_.getNrOfSegments()) {
        if (arm_chain_.getSegment(i).getName() == name) {
            return i + 1;
        }
        i++;
    }
    return -1;
}

bool ExcavaROBArmKinematics::getPositionIK(excavaROB_arm_kinematics_msgs::GetPositionIK::Request &request,
                                           excavaROB_arm_kinematics_msgs::GetPositionIK::Response &response) {
    ros::Time last_time = ros::Time::now();
    ros::Time time = ros::Time::now();
    ros::Duration dt = time - last_time;

    KDL::JntArray jnt_pos, jnt_vel;
    KDL::Frame kdl_pose;
    Eigen::VectorXd q, qd;
    Eigen::VectorXd x_desired, xd_desired;
    Eigen::VectorXd x_estimated, xd_error;

    KDL::Jacobian kdl_jacobian;
    Eigen::MatrixXd eigen_full_jacobian;
    Eigen::MatrixXd eigen_jacobian;
    Eigen::MatrixXd eigen_jjt;
    Eigen::MatrixXd eigen_jjt_inverse;
    Eigen::MatrixXd eigen_jac_pseudo_inverse;

    // variable initialization
    jnt_pos.resize(num_joints_arm_);
    jnt_vel.resize(num_joints_arm_);
    q = VectorXd::Zero(3);
    qd = VectorXd::Zero(3);
    x_desired = VectorXd::Zero(3);
    xd_desired = VectorXd::Zero(3);
    x_estimated = VectorXd::Zero(3);
    xd_error = VectorXd::Zero(3);

    kdl_jacobian.resize(4);
    eigen_full_jacobian = MatrixXd::Zero(6, 4);
    eigen_jacobian = MatrixXd::Zero(3, 3);
    eigen_jjt = MatrixXd::Zero(3, 3);
    eigen_jjt_inverse = MatrixXd::Zero(3, 3);
    eigen_jac_pseudo_inverse = MatrixXd::Zero(3, 3);

    // set cartesian position for computing of inverse kinematics
    x_desired(0) = request.pose.position.x;
    x_desired(1) = request.pose.position.y;
    x_desired(2) = request.pose.position.z;
    xd_desired(0) = request.velocity.linear.x;
    xd_desired(1) = request.velocity.linear.y;
    xd_desired(2) = request.velocity.linear.z;

    // set seed state to compute inverse kinematic algorithm based on pseudoinverse jacobian 
    for (unsigned int i = 0; i < num_joints_arm_ - 1; i++) {
	int jnt_index = getJointIndex(request.ik_seed_state.name[i]);
	if (jnt_index >= 0) {
	    ROS_DEBUG("Joint: %d %s %f %f", jnt_index, request.ik_seed_state.name[i].c_str(),
					    request.ik_seed_state.position[i], request.ik_seed_state.velocity[i]);
	    q(jnt_index) = request.ik_seed_state.position[i];
	    qd(jnt_index) = request.ik_seed_state.velocity[i];
	} else {
	    ROS_ERROR("ExcavaROB Kinematics:: It have not %s link in the chain.", request.ik_link_name.c_str());
	    response.error_code.val = response.error_code.NO_IK_SOLUTION;
	    return false;
	}
    }

    response.ik_solution.name.resize(num_joints_arm_);
    response.ik_solution.position.resize(num_joints_arm_);
    response.ik_solution.velocity.resize(num_joints_arm_);
    for (unsigned int i = 0; i < (unsigned) max_iterations_; i++) {
	if (dt.toSec() > request.ik_timeout.toSec()) {
	    ROS_ERROR("ExcavaROB Kinematics: Could not compute IK.");
	    response.error_code.val = response.error_code.NO_IK_SOLUTION;
	    return false;
	}

	for (unsigned int j = 0; j < num_joints_arm_; j++) {
	    if (j == num_joints_arm_ - 1) {
		jnt_pos(j) = request.ik_seed_state.position[j];
		jnt_vel(j) = request.ik_seed_state.velocity[j];
	    } else {
		jnt_pos(j) = q(j);
		jnt_vel(j) = qd(j);
	    }
	}

	// get cartesian pose and twist
	jnt_to_pose_solver_->JntToCart(jnt_pos, kdl_pose);
	// get the chain jacobian
	jnt_to_jac_solver_->JntToJac(jnt_vel, kdl_jacobian);

	// convert to (plain) eigen for easier math
	eigen_full_jacobian = kdl_jacobian.data;
	eigen_jacobian = eigen_full_jacobian.block(0,0,3,3);

	// compute the pseudo inverse
	eigen_jjt = eigen_jacobian * eigen_jacobian.transpose() + MatrixXd::Identity(3, 3) * pow(damping_, 2);
	eigen_jjt.computeInverse(&eigen_jjt_inverse);
	eigen_jac_pseudo_inverse = eigen_jacobian.transpose() * eigen_jjt_inverse;

	for (unsigned int j = 0; j < num_joints_arm_ - 1; j++) {
	    x_estimated(j) = kdl_pose.p(j);
	}

	xd_error = ik_gain_ * (x_desired - x_estimated) + xd_desired;

	// compute error joint velocities
	qd = eigen_jac_pseudo_inverse * xd_error;

	// integrate error joint velocities to get error joint positions
	q = q + qd * 1;//dt.toSec();

	ROS_DEBUG("iteration = %d; x_desired = [%f %f %f]; x_estimated = [%f %f %f]", 
		  i, x_desired(0), x_desired(1), x_desired(2), x_estimated(0), x_estimated(1), x_estimated(2));
	double error = sqrt((x_desired - x_estimated).dot(x_desired - x_estimated));
	if (error < eps_) {
	    for (unsigned int k = 0; k < num_joints_arm_; k++) {
		response.ik_solution.name[k] = request.ik_seed_state.name[k];
		if (k == num_joints_arm_ - 1) {
		    response.ik_solution.position[k] = request.pose.pitch;
		    response.ik_solution.velocity[k] = request.velocity.angular.x;
		} else {
		    response.ik_solution.position[k] = q(k);
		    response.ik_solution.velocity[k] = qd(k);
		}
	    }
	    response.error_code.val = response.error_code.SUCCESS;
	    break;
	} else if (i == (unsigned) max_iterations_ - 1) {
	    ROS_ERROR("ExcavaROB Kinematics: Could not compute IK.");
	    response.error_code.val = response.error_code.NO_IK_SOLUTION;
	}
	time = ros::Time::now();
	dt = time - last_time;
    }

    return true;
}


bool ExcavaROBArmKinematics::getPositionFK(excavaROB_arm_kinematics_msgs::GetPositionFK::Request &request,
                                           excavaROB_arm_kinematics_msgs::GetPositionFK::Response &response) {
    KDL::Frame p_out;
    KDL::JntArray jnt_pos_in;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

    if (num_joints_arm_ != request.fk_link_names.size()) {
	ROS_ERROR("The request has not the same dimension of the arm_chain joints.");
	return false;
    }

    jnt_pos_in.resize(num_joints_arm_);
    for (unsigned int i = 0; i < num_joints_arm_; i++) {
        int jnt_index = getJointIndex(request.joint_state.name[i]);
        if (jnt_index >= 0)
            jnt_pos_in(jnt_index) = request.joint_state.position[i];
    }

    response.pose_stamped.resize(num_joints_arm_);
    response.fk_link_names.resize(num_joints_arm_);

    bool valid = true;
    for (unsigned int i = 0; i < num_joints_arm_; i++) {
        int segmentIndex = getKDLSegmentIndex(request.fk_link_names[i]);
        ROS_DEBUG("End effector index: %d", segmentIndex);
        ROS_DEBUG("Chain indices: %d", arm_chain_.getNrOfSegments());

        if (fk_solver_->JntToCart(jnt_pos_in, p_out, segmentIndex) >= 0) {
            tf_pose.frame_id_ = root_name_;
            tf_pose.stamp_ = ros::Time();
            tf::PoseKDLToTF(p_out, tf_pose);
            try {
                tf_listener_.transformPose(request.header.frame_id, tf_pose, tf_pose);
            } catch (...) {
                ROS_ERROR("ExcavaROB Kinematics: Could not transform FK pose to frame: %s", request.header.frame_id.c_str());
                response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
                return false;
            }
            tf::poseStampedTFToMsg(tf_pose, pose);
            response.pose_stamped[i] = pose;
            response.fk_link_names[i] = request.fk_link_names[i];
            response.error_code.val = response.error_code.SUCCESS;
        } else {
            ROS_ERROR("ExcavaROB Kinematics: Could not compute FK for %s", request.fk_link_names[i].c_str());
            response.error_code.val = response.error_code.NO_FK_SOLUTION;
            valid = false;
        }
    }
    return true;
}

bool ExcavaROBArmKinematics::getKinSolverInfo(excavaROB_arm_kinematics_msgs::GetKinematicSolverInfo::Request &request,
                                              excavaROB_arm_kinematics_msgs::GetKinematicSolverInfo::Response &response) {
    response.kinematic_solver_info = info_;

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "excavaROB_arm_kinematics");
    ExcavaROBArmKinematics kin;
    if (!kin.init()) {
	ROS_ERROR("ExcavaROB Kinematics: Could not initialize kinematics node");
	return -1;
    }

    ros::spin();
    return 0;
}
