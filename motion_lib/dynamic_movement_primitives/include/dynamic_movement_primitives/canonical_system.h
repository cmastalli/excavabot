#ifndef CANONICAL_SYSTEM_H
#define CANONICAL_SYSTEM_H

#include <math.h>
//#include <ros/ros.h>

#include <dynamic_movement_primitives/CanonicalSystem.h>

//#include <dynamic_movement_primitives/dmp.h>

namespace dmp
{

class CanonicalSystem
{
  public:

    friend class DynamicMovementPrimitives;

    CanonicalSystem();

    ~CanonicalSystem();

    bool initialize(ros::NodeHandle& node_handle);

    bool initFromMessage(const dynamic_movement_primitives::CanonicalSystem& can_sys);

    bool writeToMessage(dynamic_movement_primitives::CanonicalSystem& can_sys);

    void integrateCanonicalSystem();

    void setCanonicalSystem(double alpha, double tau);


  private:

    void reset();

    bool initialized_;

    double alpha_, tau_;

    double s_, time_;


}; // @class CanonicalSystem

inline void CanonicalSystem::integrateCanonicalSystem()
{
    s_ = exp(-(alpha_ / tau_) * time_);
}

inline void CanonicalSystem::reset()
{
    s_ = 1.0;
    time_ = 0.0;
}

inline void CanonicalSystem::setCanonicalSystem(double alpha, double tau)
{
    alpha_ = alpha;
    tau_ = tau;
}


} // @namespace dmp

#endif /* CANONICAL_SYSTEM_H */
