#include <ros/node_handle.h>

#include <dynamic_movement_primitives/canonical_system.h>

namespace dmp
{

CanonicalSystem::CanonicalSystem() :
    initialized_(false), alpha_(0.0), tau_(0.0), s_(1.0), time_(0.0)
{
}

CanonicalSystem::~CanonicalSystem() {
}


bool CanonicalSystem::initialize(ros::NodeHandle& node_handle)
{
    if (initialized_)
        ROS_WARN("Canonical system already initialized. Reinitializing with new parameters.");

    // get alpha
    ros::NodeHandle nh(node_handle, "canonical_system");
    if (!nh.getParam("alpha", alpha_)) {
        ROS_ERROR("Could not get alpha from parameter server.");
        initialized_ = false;
        return initialized_;
    }
    else if (alpha_ < 0) {
        ROS_ERROR("Alpha value must be positive.");
        initialized_ = false;
        return initialized_;
    }
    // get tau
    if (!nh.getParam("tau", tau_)) {
        ROS_ERROR("Could not get tau from parameter server.");
        initialized_ = false;
        return initialized_;
    }
    else if (tau_ < 0) {
        ROS_ERROR("Tau value must be positive.");
        initialized_ = false;
        return initialized_;
    }

    initialized_ = true;
    return initialized_;
}

bool CanonicalSystem::initFromMessage(const dynamic_movement_primitives::CanonicalSystem& can_sys)
{
    initialized_ = can_sys.initialized;
    tau_ = can_sys.tau;
    alpha_ = can_sys.alpha;

    return true;
}

bool CanonicalSystem::writeToMessage(dynamic_movement_primitives::CanonicalSystem& can_sys)
{
    if (!initialized_) {
        ROS_ERROR("Could not write because it need initialized the canonical system.");
        return initialized_;
    }

    can_sys.initialized = initialized_;
    can_sys.tau = tau_;
    can_sys.alpha = alpha_;

    return true;
}

} //@namespace dmp
