#ifndef TRANSFORMATION_SYSTEM_H_
#define TRANSFORMATION_SYSTEM_H_

#include <ros/ros.h>

#include <dynamic_movement_primitives/dmp.h>
#include <dynamic_movement_primitives/TransformationSystem.h>
#include <lwr/lwr.h>

// system includes
#include <string>
#include <vector>
#include <algorithm>


namespace dmp
{

class TransformationSystem
{

  public:

    friend class DynamicMovementPrimitives;

    TransformationSystem();

    ~TransformationSystem();

    bool initialize(ros::NodeHandle& node_handle, int trans_sys_id);

    bool initFromMessage(const dynamic_movement_primitives::TransformationSystem& trans_sys);

    bool writeToMessage(dynamic_movement_primitives::TransformationSystem& trans_sys);


  private:

    void reset();

    double getState() const;

    double getGoal() const;

    void setState(const double x,
                  const double v);

    void setStart(const double x0);

    void setGoal(const double goal);

    void setInitialStart(const double initial_x0);

    void setInitialGoal(const double initial_goal);


    bool initialized_;

    int trans_sys_id_;

    double k_gain_, d_gain_;

    lwr::LocallyWeightedRegression lwr_model_;

    std::vector<double> target_function_;

    /*! Internal states
     */
    double v_; //vel
    double vd_; //acc

    /*! External states
     */
    double x_; //pos
    double xd_; //vel
    double xdd_; //acc

    /*! The targets state used during supervised learning
     */
    double xt_;
    double xtd_;
    double xtdd_;

    /*! The start state
     */
    double x0_;
    double initial_x0_;

    /*! The goal state
     */
    double goal_;
    double initial_goal_;

    /*! The current values of the nonlinear function
     */
    double f_;
    double ft_;


}; // @class TransformationSystem

inline void TransformationSystem::reset()
{
  v_ = 0.0;
  vd_ = 0.0;
  x_ = 0.0;
  xd_ = 0.0;
  xdd_ = 0.0;
}

inline double TransformationSystem::getState() const
{
  return x_;
}

inline double TransformationSystem::getGoal() const
{
  return goal_;
}

inline void TransformationSystem::setState(const double x,
                                           const double v)
{
  x_ = x;
  v_ = v;
}

inline void TransformationSystem::setStart(const double x0)
{
  x0_ = x0;
}

inline void TransformationSystem::setGoal(const double goal)
{
  goal_ = goal;
}

inline void TransformationSystem::setInitialStart(const double initial_x0)
{
  initial_x0_ = initial_x0;
}

inline void TransformationSystem::setInitialGoal(const double initial_goal)
{
  initial_goal_ = initial_goal;
}

} // @namespace dmp

#endif /* TRANSFORMATION_SYSTEM_H_ */
