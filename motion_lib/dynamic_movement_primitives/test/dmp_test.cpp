
#include <dynamic_movement_primitives/dmp.h>
//#include <dynamic_movement_primitives/trajectory.h>

#include <Eigen/Eigen>


using namespace Eigen;
using namespace dmp;

class DMPTest
{
  public:
    DMPTest(ros::NodeHandle& node_handle);

    ~DMPTest();

    bool initialize();

    bool runTrajectoryTest();

    bool runDemonstrationLearningTest();


  private:
    ros::NodeHandle nh_private_;

    Eigen::VectorXd end_pos_;

    Eigen::VectorXd start_pos_;

    std::string dmp_name_, directory_name_;

    double step_duration_, execution_duration_;

    bool change_execution_duration_;


}; // @class DMPTest


DMPTest::DMPTest(ros::NodeHandle& node_handle) :
    nh_private_(node_handle, "dmp_test/dynamic_movement_primitives")
{
}


DMPTest::~DMPTest() {
}


bool DMPTest::initialize()
{
    ros::NodeHandle nh1(nh_private_, "dmp");
    if (!nh1.getParam("dmp_name", dmp_name_)) {
        ROS_ERROR("Could not get dmp_name.");
        return false;
    }
    if (!nh1.getParam("step_duration", step_duration_)) {
	step_duration_ = 0.035;
        ROS_WARN("Could not get step_duration. Therefore, step_duration is set to 0.035.");
    }
    if (step_duration_ == 0) {
	step_duration_ = 0.035;
        ROS_INFO("It change step_duration value to 0.035.");
    }

    if (!nh1.getParam("execution_duration", execution_duration_)) {
        ROS_WARN("Could not get step_duration. Therefore, execution_duration is set to original values.");
	change_execution_duration_ = true;
    }

    ros::NodeHandle nh2(nh_private_, "lwr");
    if (!nh2.getParam("path_name", directory_name_)) {
        ROS_ERROR("Could not get dmp_name.");
        return false;
    }

    return true;
}


bool DMPTest::runTrajectoryTest()
{
    dmp::Trajectory dmp_traj;
    if (!dmp_traj.initialize(nh_private_)) {
        ROS_ERROR("Could not initialize trajectory of DMP.");
        return false;
    }

    // test readFromFile
    if (!dmp_traj.readFromFile()) {
        ROS_ERROR("Could not read the trajectory.");
        return false;
    }

    if (execution_duration_ == 0 || change_execution_duration_) {
	execution_duration_ = dmp_traj.getDuration();
        ROS_INFO("It change execution_duration to original value.");
    }

    // test writeFromFile
    std::string file_write_name = dmp_name_ + std::string("_writed_action.txt");
    if (!dmp_traj.writeToFile(file_write_name)) {
        ROS_ERROR("Could not write the trajectory.");
        return false;
    }

    // test getTrajectory
    double pos_x = dmp_traj.getTrajectory(1, "x");
    double vel_y = dmp_traj.getTrajectory(2, "yd");
    double acc_z =  dmp_traj.getTrajectory(4, "zdd");
    ROS_INFO("[x(%i) yd(%i) zdd(%i)] = [%f %f %f]", 1, 2, 4, pos_x, vel_y, acc_z);

    // test getStartPosition
    int traj_dim = dmp_traj.getDimension();
    start_pos_ = VectorXd::Zero(traj_dim);
    if (dmp_traj.getStartPosition(start_pos_))
        for (int i = 0; i < start_pos_.size(); i++)
            ROS_INFO("x0(%i) = %f", i, start_pos_(i));

    // test getEndPosition
    end_pos_ = VectorXd::Zero(traj_dim);
    if (dmp_traj.getEndPosition(end_pos_))
        for (int i = 0; i < end_pos_.size(); i++)
	    ROS_INFO("xf(%i) = %f", i, end_pos_(i));

    double time_duration = dmp_traj.getDuration();
    ROS_INFO("Time duration: %f", time_duration);
 
    return true;
}


bool DMPTest::runDemonstrationLearningTest()
{
    dmp::DynamicMovementPrimitives dmp(nh_private_);
    if (!dmp.initialize()) {
        ROS_ERROR("Could not initialize the DMP.");
        return false;
    }

    if (!dmp.learnFromTrajectory()) {
        ROS_ERROR("Could not learn from trajectory.");
        return false;
    }

    if (!dmp.writeToFile(directory_name_)) {
        ROS_ERROR("Could not write to file.");
        return false;
    }

    if (!dmp.writeToDisc(directory_name_)) {
        ROS_ERROR("Could not write to disc.");
        return false;
    }

    dmp::DynamicMovementPrimitives dmp_copy(nh_private_);
    if (!dmp_copy.initializeFromDisc(directory_name_, dmp_name_)) {
        ROS_ERROR("Could not initialized from disc.");
        return false;
    }

    double num_trans_sys = dmp_copy.getNumOfDMPs();
    VectorXd start = VectorXd::Zero(num_trans_sys);
    VectorXd goal = VectorXd::Zero(num_trans_sys);


    // left front truck position
    // trajectory generated 1
    for (int i = 0; i < num_trans_sys; i++)
        start(i) = start_pos_(i);
    goal(0) = end_pos_(0) - 1;
    goal(1) = end_pos_(1) - 1;
    goal(2) = end_pos_(2) + 1;
    goal(3) = end_pos_(3) + 0.5;

    dmp_copy.changeExecutionTime(execution_duration_);

    if (!dmp_copy.generateTrajectory(start, goal, step_duration_)) {
        ROS_ERROR("Could not generate the copy trajectory 1.");
        return false;
    }
    if (!dmp_copy.writeToFile(directory_name_ + "1_")) {
        ROS_ERROR("Could not write the copy to files.");
        return false;
    }

    // trajectory generated 2
    goal(0) = end_pos_(0) + 1;
    goal(1) = end_pos_(1) + 1;
    goal(2) = end_pos_(2) - 1;
    goal(3) = end_pos_(3) - 0.5;
    if (!dmp_copy.generateTrajectory(start, goal, step_duration_)) {
    	ROS_ERROR("Could not generate the copy trajectory 2.");
    	return false;
    }
    if (!dmp_copy.writeToFile(directory_name_ + "2_")) {
    	ROS_ERROR("Could not write the copy to files.");
    	return false;
    }

  // trajectory generated 3
    start(0) = start_pos_(0) - 1;
    start(1) = start_pos_(1) - 1;
    start(2) = start_pos_(2) + 1;
    start(3) = start_pos_(3) + 0.5;
    if (!dmp_copy.generateTrajectory(start, end_pos_, step_duration_)) {
    	ROS_ERROR("Could not generate the copy trajectory 3.");
    	return false;
    }
    if (!dmp_copy.writeToFile(directory_name_ + "3_")) {
    	ROS_ERROR("Could not write the copy to files.");
    	return false;
    }

  // trajectory generated 4
    start(0) = start_pos_(0) + 1;
    start(1) = start_pos_(1) + 1;
    start(2) = start_pos_(2) - 1;
    start(3) = start_pos_(3) - 0.5;
    if (!dmp_copy.generateTrajectory(start, end_pos_, step_duration_)) {
    	ROS_ERROR("Could not generate the copy trajectory 4.");
    	return false;
    }
    if (!dmp_copy.writeToFile(directory_name_ + "4_")) {
    	ROS_ERROR("Could not write the copy to files.");
    	return false;
    }


  return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "dmp_test");

  ros::NodeHandle node_handle;
  DMPTest dmp_test(node_handle);
  if (!dmp_test.initialize()) {
    ROS_ERROR("Could not initialize DMP test node.");
    return -1;
  }

  if (!dmp_test.runTrajectoryTest()) {
    ROS_ERROR("Could run trajectory test.");
    return -1;
  }

  if (!dmp_test.runDemonstrationLearningTest()) {
    ROS_ERROR("Could not run demonstration learning test.");
    return -1;
  }


  return 0;
}

