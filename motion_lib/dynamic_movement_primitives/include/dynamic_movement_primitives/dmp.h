#ifndef DYNAMIC_MOVEMENT_PRIMITIVES_H
#define DYNAMIC_MOVEMENT_PRIMITIVES_H

//#include <ros/ros.h>

// local includes
#include <dynamic_movement_primitives/trajectory.h>
#include <dynamic_movement_primitives/transformation_system.h>
#include <dynamic_movement_primitives/canonical_system.h>
#include <dynamic_movement_primitives/Model.h>

#include <Eigen/Eigen>


namespace dmp
{

class TransformationSystem;
class CanonicalSystem;

static const int MIN_NUM_DATA_POINTS = 90;


class DynamicMovementPrimitives
{
  public:

    DynamicMovementPrimitives(ros::NodeHandle& node_handle);

    ~DynamicMovementPrimitives();

    bool initialize();

    bool initializeFromDisc(const std::string directory_name,
                            const std::string dmp_name);

    bool initFromMessage(const dynamic_movement_primitives::Model& dmp_model);

    bool writeToDisc(const std::string directory_name);

    bool writeToMessage(dynamic_movement_primitives::Model& dmp_model);

    bool writeToFile(std::string directory_name);

    bool writeTargetFunctionData(std::string file_path);

    bool writeTrajectoryGenerated(std::string file_path);

    bool learnFromTrajectory();

    bool generateTrajectory(Eigen::VectorXd x0, Eigen::VectorXd goal, const double step_duration);

    bool generateStepTrajectory(Eigen::MatrixXd& traj_desired, bool& movement_finished, const double step_duration);

    bool setup(Eigen::VectorXd x0, Eigen::VectorXd goal);

    bool setup(Eigen::VectorXd goal);

    bool changeStartState(const Eigen::VectorXd& x0);

    bool changeGoalState(const Eigen::VectorXd& goal);

    void changeExecutionTime(double execution_time);

    double getNumOfDMPs() const;

    bool isStartStateSet() const;

    std::string getDMPId() const;


  private:

    ros::NodeHandle node_handle_;

    void computeTargetFunction();

    bool learnTargetFunction();

    bool computeTargetFunctionLearned();

    std::string int2string(int i);



    bool initialized_;

    bool initialized_by_msg_;

    int num_trans_sys_;

    bool is_setup_, is_start_state_set_;

    dmp::Trajectory trajectory_;

    std::vector<dmp::TransformationSystem> transformation_systems_;

    dmp::CanonicalSystem canonical_system_;

    std::vector<std::string> traj_names_;

    double delta_t_;

    Eigen::MatrixXd ft_learned_;

    std::vector<double> t_;

    std::vector<std::vector <double> > x_, xd_, xdd_;

    std::vector<double> ft_input_;

    bool is_learned_;

    std::string path_name_;

    std::string dmp_name_;

    double execution_duration_;

}; // @class DynamicMovementPrimitives


inline std::string DynamicMovementPrimitives::int2string(int i)
{
    ostringstream temp;
    temp << i;

    return temp.str();
}


inline double DynamicMovementPrimitives::getNumOfDMPs() const
{
    return num_trans_sys_;
}


inline bool DynamicMovementPrimitives::isStartStateSet() const
{
    return is_start_state_set_;
}


inline std::string DynamicMovementPrimitives::getDMPId() const
{
    return dmp_name_;
}


} // @namespace dmp


#endif /* DYNAMIC_MOVEMENT_PRIMITIVES_H */
