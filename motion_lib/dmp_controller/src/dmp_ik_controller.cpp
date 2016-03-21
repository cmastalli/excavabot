
#include <dmp_controller/dmp_ik_controller.h>

#include <angles/angles.h>

//#include <kdl/chainfksolvervel_recursive.hpp>
//#include <kdl/chainiksolvervel_wdls.hpp>
//#include <kdl/kinfam_io.hpp>

#include <pluginlib/class_list_macros.h>


PLUGINLIB_DECLARE_CLASS(dmp_controller, InverseKinematicsController, dmp_controller::InverseKinematicsController, pr2_controller_interface::Controller)


namespace dmp_controller
{

InverseKinematicsController::InverseKinematicsController() : 
    robot_(NULL), movement_finished_(false), jnt_to_jac_solver_(NULL), jnt_to_pose_solver_(NULL)//,
//    jnt_to_twist_solver_(NULL)
{
//    pthread_mutex_init(&dmp_ik_controller_lock_, NULL);
}


InverseKinematicsController::~InverseKinematicsController()
{
}


bool InverseKinematicsController::init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle& node_handle)
{
    using namespace XmlRpc;

    assert(robot);
    robot_ = robot;
    node_handle_ = node_handle;

    // Gets all of the joints
    XmlRpc::XmlRpcValue joint_names;
    if (!node_handle_.getParam("joints", joint_names)) {
	ROS_ERROR("No joints given in namespace: %s.", node_handle_.getNamespace().c_str());
	return false;
    }

    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
	ROS_ERROR("Malformed joint specification.");
	return false;
    }

    for (int i = 0; i < joint_names.size(); ++i) {
	XmlRpcValue &name_value = joint_names[i];
	if (name_value.getType() != XmlRpcValue::TypeString) {
 	    ROS_ERROR("Array of joint names should contain all strings.");
	    return false;
	}

	pr2_mechanism_model::JointState *joint_state = robot->getJointState((std::string)name_value);
	if (!joint_state) {
	    ROS_ERROR("Joint %s not found in namespace: %s.", ((std::string) name_value).c_str(), node_handle_.getNamespace().c_str());
	return false;
	}
	joint_states_.push_back(joint_state);
	joint_names_.push_back((std::string) name_value);
    }

    // Ensures that all the joints are calibrated.
    for (size_t i = 0; i < joint_states_.size(); ++i) {
	if (!joint_states_[i]->calibrated_) {
	    ROS_ERROR("Joint %s was not calibrated (namespace: %s)", joint_states_[i]->joint_->name.c_str(), node_handle_.getNamespace().c_str());
	    return false;
	}
    }

    // Sets up pid controllers for all of the joints
    std::string gains_ns;
    if (!node_handle_.getParam("gains", gains_ns))
	gains_ns = node_handle_.getNamespace() + "/gains";

    pids_.resize(joint_states_.size());
    for (size_t i = 0; i < joint_states_.size(); ++i)
	if (!pids_[i].init(ros::NodeHandle(gains_ns + "/" + joint_names_[i])))
	    return false;
    commanded_effort_ = VectorXd::Zero(4);

    if (!dmp_controller_.initialize(node_handle_)) {
	ROS_ERROR("Could not initialized dmp controller.");
	return false;
    }

//    num_trans_sys_ = 4;//dmp_->getNumOfDMPs();
    start_ = VectorXd::Zero(joint_states_.size());
    goal_ = VectorXd::Zero(joint_states_.size());
    traj_desired_ = MatrixXd::Zero(joint_states_.size(), 3);//POS_VEL_ACC);

    if (!initKinematic()) {
	ROS_ERROR("Could not initialized kinematic.");
	return false;
    }

    // get current state
    Eigen::VectorXd pose_desired(joint_states_.size());
    if (!getCurrentState(pose_desired)) {
	ROS_ERROR("Could not starting controller because it can't get current state.");
	return false;
    }
    // set with desired trajectory
    for (size_t i = 0; i < joint_states_.size(); i++)
	traj_desired_(i, 0) = pose_desired(i);

    initPublisher();

    return true;
}


void InverseKinematicsController::starting()
{
    last_time_ = robot_->getTime();

    // reset PID joint controllers
    for (size_t i = 0; i < pids_.size(); ++i)
	pids_[i].reset();
}


void InverseKinematicsController::update()
{
    ros::Time time = robot_->getTime();
    dt_ = time - last_time_;
    last_time_ = time;

    if (dmp_controller_.tryMutexLock()) {
	if (!getDMPStepTrajectory(dt_.toSec())) {
	    // do something
	    ROS_INFO("Finished movement of DMP name id %s", dmp_controller_.getDMPName().c_str());
	    dmp_controller_.incrementDMPQueueIndex();
	}
	else {
	    //setDMPCommand(traj_desired_); creo que no hace falta pues voy a corre la ik y control aqui!
	    //visualize();
	}
	dmp_controller_.freeMutexLock();
    }
    else {
	dmp_controller_.freeMutexLock();
	dmp_controller_.incrementSkippedCycles();
    }

    computeInverseKinematic();

    computeControlCommand();

    publishStateController(time);
}


bool InverseKinematicsController::initKinematic()
{
    if (!node_handle_.getParam("root_name", root_name_)) {
	ROS_ERROR("No root name found on parameter server (namespace: %s).", node_handle_.getNamespace().c_str());
	return false;
    }

    if (!node_handle_.getParam("tip_name", tip_name_)) {
	ROS_ERROR("No tip name found on parameter server (namespace: %s).", node_handle_.getNamespace().c_str());
	return false;
    }

    // create robot chain from root to tip
    if (!chain_.init(robot_, root_name_, tip_name_))
	return false;

    if (!chain_.allCalibrated()) {
	ROS_ERROR("Not all joints in the chain are calibrated.");
	return false;
    }
    chain_.toKDL(kdl_chain_);

    if (!node_handle_.getParam("damping_factor", damping_)) {
	ROS_WARN("The damping factor is equal to 0.");
	damping_ = 0;
    }

    if (!node_handle_.getParam("k_ik", gain_ik_)) {
	ROS_WARN("Not found the k gain of inverse kinematic algorithm.");
	return false;
    }

    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
//    jnt_to_twist_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));

    q_.resize(joint_states_.size());
    kdl_chain_jacobian_.resize(joint_states_.size());

    full_jacobian_ = MatrixXd::Zero(6, joint_states_.size());
    eigen_chain_jacobian_ = MatrixXd::Zero(3, 3);
    eigen_jjt_ = MatrixXd::Zero(3, 3);
    eigen_jjt_inverse_ = MatrixXd::Zero(3, 3);
    eigen_jac_pseudo_inverse_ = MatrixXd::Zero(3, 3);

    x_desired_ = VectorXd::Zero(3);
    xd_desired_ = VectorXd::Zero(3);

    x_estimated_ = VectorXd::Zero(3);
    xd_error_ = VectorXd::Zero(3);

    q_desired_ = VectorXd::Zero(3);
    qd_desired_ = VectorXd::Zero(3);

    q_error_ = VectorXd::Zero(4);
    qd_error_ = VectorXd::Zero(4);

    return true;
}


void InverseKinematicsController::initPublisher()
{
    state_publish_time_ = 0.1;
    controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<DMPJointControllerState> (node_handle_, "state", 1));

    controller_state_publisher_->lock();
    for (size_t j = 0; j < joint_states_.size(); ++j)
	controller_state_publisher_->msg_.joint_names.push_back(joint_states_[j]->joint_->name);

    controller_state_publisher_->msg_.q_desired.resize(joint_states_.size());
    controller_state_publisher_->msg_.qd_desired.resize(joint_states_.size());

    controller_state_publisher_->msg_.q_actual.resize(joint_states_.size());
    controller_state_publisher_->msg_.qd_actual.resize(joint_states_.size());

    controller_state_publisher_->msg_.q_error.resize(joint_states_.size());
    controller_state_publisher_->msg_.qd_error.resize(joint_states_.size());

    controller_state_publisher_->msg_.u.resize(joint_states_.size());
    controller_state_publisher_->unlock();
}


bool InverseKinematicsController::getDMPStepTrajectory(const double step_duration)
{
    if (!dmp_controller_.getCurrentDMP()->isStartStateSet()) {
	// get current state and change start state at DMP
	Eigen::VectorXd start_state(joint_states_.size());
	if (!getCurrentState(start_state)) {
	    ROS_WARN("Could not change the start state. The DMP added was eliminated!");
	    return false;
	}
	ROS_INFO("Setting start state in (%f %f %f %f).", start_state(0), start_state(1), start_state(2), start_state(3));
	dmp_controller_.getCurrentDMP()->changeStartState(start_state);
    }

    // TODO: include a subscriptor of goal message because it's import to change the goal state in a online way

    // set time in the first DMP cycle
    if (!dmp_controller_.isFirstDMPCycle()) {
	dmp_controller_.setStartTime(ros::Time::now());
    }

    if (!dmp_controller_.getCurrentDMP()->generateStepTrajectory(traj_desired_, movement_finished_, step_duration))
	ROS_WARN("Could not generate the trajectory in step.");

    if (movement_finished_) {
	movement_finished_ = false;
	dmp_controller_.setEndTime(ros::Time::now());
	return movement_finished_;
    }

    return true;
}


void InverseKinematicsController::computeInverseKinematic()
{
    // set desired pose
    for (size_t i = 0; i < joint_states_.size() - 1; i++) {
	x_desired_(i) = traj_desired_(i, 0);
	xd_desired_(i) = traj_desired_(i, 1);
    }

    // get cartesian pose
    jnt_to_pose_solver_->JntToCart(q_, kdl_pose_);

    // get the chain jacobian
    jnt_to_jac_solver_->JntToJac(q_, kdl_chain_jacobian_);

    // set estimated pose
    for (size_t i = 0; i < joint_states_.size() - 1; i++) {
	q_(i) = q_desired_(i);
	x_estimated_(i) = kdl_pose_.p(i);
    }
    q_(3) = traj_desired_(3, 0);

    // compute error of estimated twist
    xd_error_ = gain_ik_ * (x_desired_ - x_estimated_) + xd_desired_;

    // convert to (plain) eigen for easier math
    full_jacobian_ = kdl_chain_jacobian_.data;
    eigen_chain_jacobian_ = full_jacobian_.block(0, 0, 3, 3);

    // compute the pseudo inverse
    eigen_jjt_ = eigen_chain_jacobian_ * eigen_chain_jacobian_.transpose() + MatrixXd::Identity(3, 3) * pow(damping_, 2);
    eigen_jjt_inverse_ = eigen_jjt_.inverse();
    eigen_jac_pseudo_inverse_ = eigen_chain_jacobian_.transpose() * eigen_jjt_inverse_;

    // compute error joint velocities
    qd_desired_ = eigen_jac_pseudo_inverse_ * xd_error_;

    // integrate error joint velocities to get error joint positions
    q_desired_ = q_desired_ + qd_desired_ * dt_.toSec();

    // get error joint pose and twist for pid controllers
    for (size_t i = 0; i < joint_states_.size(); i++) {
	if (i == joint_states_.size() - 1) {
	    q_error_(i) = traj_desired_(i, 0) - joint_states_[i]->position_;
	    qd_error_(i) = traj_desired_(i, 1) - joint_states_[i]->velocity_;
	}
	else {
	    q_error_(i) = q_desired_(i) - joint_states_[i]->position_;
	    qd_error_(i) = qd_desired_(i) - joint_states_[i]->velocity_;
	}
    }
}


void InverseKinematicsController::computeControlCommand()
{
    // compute the control signals
    for (size_t i = 0; i < joint_states_.size(); i++) {
	commanded_effort_(i) = pids_[i].updatePid(-q_error_(i), -qd_error_(i), dt_);
	    if (x_desired_(0) != 0)
	    joint_states_[i]->commanded_effort_ = commanded_effort_(i);
    }
}


bool InverseKinematicsController::getCurrentState(Eigen::VectorXd& state)
{
    chain_.getPositions(q_);
    jnt_to_pose_solver_->JntToCart(q_, kdl_pose_);

    if (state.size() != (int)joint_states_.size()) {
	ROS_ERROR("Could not get the current state because it's no the same size.");
	return false;
    }

    for (size_t i = 0; i < joint_states_.size(); i++) {
	if (i == joint_states_.size() - 1)
	    state(i) = q_(i);
	else
	    state(i) = kdl_pose_.p(i);
    }

    return true;
}


void InverseKinematicsController::publishStateController(const ros::Time& time)
{
    if ((time - last_publish_time_).toSec() < state_publish_time_)
	return;

    if (controller_state_publisher_->trylock()) {
	controller_state_publisher_->msg_.header.stamp = time;
	for (size_t i = 0; i < joint_states_.size(); i++) {
	    if (i == joint_states_.size() - 1) {
		controller_state_publisher_->msg_.q_desired[i] = traj_desired_(i, 0);
		controller_state_publisher_->msg_.qd_desired[i] = traj_desired_(i, 1);
	    }
	    else {
	    controller_state_publisher_->msg_.q_desired[i] = q_desired_(i);
	    controller_state_publisher_->msg_.qd_desired[i] = qd_desired_(i);
	    }
	// to publish the actual joint positions
	controller_state_publisher_->msg_.q_actual[i] = joint_states_[i]->position_;
	controller_state_publisher_->msg_.qd_actual[i] = joint_states_[i]->velocity_;
	// to publish the error joint positions
	controller_state_publisher_->msg_.q_error[i] = q_error_(i);
	controller_state_publisher_->msg_.qd_error[i] = qd_error_(i);
	// to publish the joint command effort
	controller_state_publisher_->msg_.u[i] = commanded_effort_(i);
	}
    }
    controller_state_publisher_->unlockAndPublish();
    last_publish_time_ = time;
}


} // @namespace dmp_controller
