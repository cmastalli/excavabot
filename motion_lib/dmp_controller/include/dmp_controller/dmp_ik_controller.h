
#include <ros/node_handle.h>

#include <dmp_controller/dmp_controller.h>
#include <dynamic_movement_primitives/dmp.h>

#include <pr2_controller_interface/controller.h>

#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/chain.h>

#include <control_toolbox/pid.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>

#include <realtime_tools/realtime_publisher.h>

// messages
#include <dmp_controller/DMPJointControllerState.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "kdl/chainfksolverpos_recursive.hpp"

#include <Eigen/Eigen>


using namespace Eigen;

namespace dmp_controller
{

class InverseKinematicsController : public pr2_controller_interface::Controller
{
  public:
    InverseKinematicsController();

    ~InverseKinematicsController();

    bool init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle& node_handle);

    void starting();

    void update();

//    pthread_mutex_t dmp_ik_controller_lock_;



  private:

    bool initKinematic();

    void initPublisher();

    bool getDMPStepTrajectory(const double step_duration);

    void computeInverseKinematic();

    void computeControlCommand();

    bool getCurrentState(Eigen::VectorXd& state);

    void publishStateController(const ros::Time& time);



    ros::NodeHandle node_handle_;

    ros::Duration dt_;

    ros::Time last_time_;

    pr2_mechanism_model::RobotState *robot_;

    pr2_mechanism_model::Chain chain_;

    std::vector<pr2_mechanism_model::JointState*> joint_states_;

    std::vector<control_toolbox::Pid> pids_;

    DMPController dmp_controller_;

    boost::scoped_ptr<realtime_tools::RealtimePublisher<dmp_controller::DMPJointControllerState> >  controller_state_publisher_;

    Eigen::VectorXd start_, goal_;

    Eigen::MatrixXd traj_desired_;

    std::string root_name_, tip_name_;

    std::vector<std::string> joint_names_;

    double num_trans_sys_;

    bool movement_finished_;

    KDL::Chain kdl_chain_;

    KDL::Jacobian kdl_chain_jacobian_;

    KDL::JntArray q_;

    KDL::Frame kdl_pose_;

//    KDL::Twist kdl_twist_;

    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

    boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;

//    boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;

    Eigen::MatrixXd full_jacobian_, eigen_chain_jacobian_;

    Eigen::MatrixXd eigen_jjt_, eigen_jjt_inverse_, eigen_jac_pseudo_inverse_;

    Eigen::VectorXd x_desired_, xd_desired_;

    Eigen::VectorXd x_estimated_, xd_error_;

    Eigen::VectorXd q_desired_, qd_desired_;

    Eigen::VectorXd q_error_, qd_error_;

    Eigen::VectorXd commanded_effort_;

    double damping_, gain_ik_;

    ros::Time last_publish_time_;

    double state_publish_time_;


};  // @class InverseKinematicsController

} // @namespace dmp_controller

