#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <ros/ros.h>

#include <dynamic_movement_primitives/Trajectory.h>

#include <fstream>

#include <Eigen/Eigen>

namespace dmp
{
  using namespace std;
  static const int POS_VEL_ACC = 3;


class Trajectory
{
  public:
    Trajectory();

    ~Trajectory();

    bool initialize(ros::NodeHandle& node_handle);

    bool initFromMessage(const dynamic_movement_primitives::Trajectory& traj_msg);

    bool readFromFile();

    bool writeToMessage(dynamic_movement_primitives::Trajectory& traj_msg);

    bool writeToFile(std::string file_name);

    double getTrajectory(int trajectory_index,
                         std::string type_variable) const;

    bool getStartPosition(Eigen::VectorXd &start_position) const;

    bool getEndPosition(Eigen::VectorXd &end_position) const;

    int getDimension() const;

    int getLength() const;

    int getSamplingFrequency();

    void getTrajectoryNames(std::vector<std::string>& names);

    double getDuration();

    double getTime(int trajectory_index) const;


  private:
    double getTrajectoryDuration();

    Eigen::VectorXd time_data_;

    Eigen::MatrixXd motion_data_;

    std::vector<std::string> position_names_, velocity_names_, acceleration_names_;

    std::string path_name_, file_name_;

    bool initialized_;

    int trajectory_length_, trajectory_dimension_;

    double time_duration_;

}; // @class Trajectory


// inline functions
inline double Trajectory::getTime(int trajectory_index) const
{
    return (double) time_data_(trajectory_index);
}


inline double Trajectory::getTrajectory(int trajectory_index,
                                        std::string type_variable) const
{
  int type_index = 0;
  for (int i = 0; i < trajectory_dimension_; i++)
  {
    if (type_variable == position_names_[i])
    {
      type_index = i;
      break;
    }
    else if (type_variable == velocity_names_[i])
    {
      type_index = i + trajectory_dimension_;
      break;
    }
    else if (type_variable == acceleration_names_[i])
    {
      type_index = i + 2 * trajectory_dimension_;
      break;
    }
    else if (i == trajectory_dimension_ - 1)
      ROS_ERROR("Could not found the trajectory for %s", type_variable.c_str());
  }

  return motion_data_(trajectory_index, type_index);
}


inline bool Trajectory::getStartPosition(Eigen::VectorXd &start_position) const
{
    if (start_position.size() == trajectory_dimension_) {
        for (int i = 0; i < trajectory_dimension_; i++)
	    start_position(i) = motion_data_(0, i);
  	}
	else {
	    ROS_INFO("Could not get the start position because the dimension is wrong");
	    return false;
  	}

    return true;
}


inline bool Trajectory::getEndPosition(Eigen::VectorXd &end_position) const
{
    if (end_position.size() == trajectory_dimension_) {
	for (int i = 0; i < trajectory_dimension_; i++)
      	end_position(i) = motion_data_(trajectory_length_ - 1, i);
    }
    else {
    	ROS_INFO("Could not get the end position because the dimension is wrong");
    	return false;
    }

    return true;
}


inline int Trajectory::getDimension() const
{
    return trajectory_dimension_;
}

inline int Trajectory::getLength() const
{
    return trajectory_length_;
}

inline double Trajectory::getDuration()
{
    return time_duration_;
}


} // @namespace dmp

#endif /* TRAJECTORY_H */
