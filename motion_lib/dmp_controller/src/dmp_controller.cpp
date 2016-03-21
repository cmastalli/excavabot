
#include <dmp_controller/dmp_controller.h>

namespace dmp_controller
{

const int SIZE_RUN_QUEUE = 100;

DMPController::DMPController()
{
    if (pthread_mutex_init(&dmp_cmd_lock_, NULL) != 0)
	ROS_ERROR("Could not initialize mutex (%i : %s).", pthread_mutex_init(&dmp_cmd_lock_, NULL), strerror(pthread_mutex_init(&dmp_cmd_lock_, NULL)));

    if (pthread_mutex_unlock(&dmp_cmd_lock_) != 0)
	ROS_ERROR("Could not unlock mutex (%i : %s).", pthread_mutex_init(&dmp_cmd_lock_, NULL), strerror(pthread_mutex_init(&dmp_cmd_lock_, NULL)));
}


DMPController::~DMPController()
{
}


bool DMPController::initialize(ros::NodeHandle node_handle)
{
    node_handle_ = node_handle;

    // advertise service
    add_to_execute_dmp_queue_server_ = node_handle_.advertiseService("add_to_execute_dmp_queue", &DMPController::addToExecuteDMPQueue, this);
//    write_trajectories_service_server_ = node_handle_.advertiseService("write_trajectories", &DMPController::writeTrajectories, this);

    // pre-allocate memory
    run_buffer_.set_capacity(SIZE_RUN_QUEUE);

    is_first_dmp_cycle_ = true;
    return true;
}


bool DMPController::addToExecuteDMPQueue(dmp_controller::AddToExecuteDMPQueue::Request& request, dmp_controller::AddToExecuteDMPQueue::Response& response)
{
    std::vector<DMPStruct> dmp_structs;

    if (!setDMPStructFromRequest(node_handle_, request, response, dmp_structs)) {
	ROS_ERROR("Setting dmp structs failed.");
	response.info.assign(std::string("Setting dmp structs failed."));
	response.return_code = dmp_controller::AddToExecuteDMPQueue::Response::SERVICE_CALL_FAILED;
	return true;
    }

    addToQueue(dmp_structs);

    response.info.assign("Starting dmps.");
    response.return_code = dmp_controller::AddToExecuteDMPQueue::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
}


bool DMPController::setDMPStructFromRequest(ros::NodeHandle& node_handle, dmp_controller::AddToExecuteDMPQueue::Request& request,
					    dmp_controller::AddToExecuteDMPQueue::Response& response, std::vector<DMPStruct>& dmp_structs)
{
    dmp_structs.clear();
    for (int i = 0; i < static_cast<int>(request.dmps_model.size()); i++) {
	DMPStruct dmp_struct;
	dmp_struct.dmp.reset(new dmp::DynamicMovementPrimitives(node_handle));

	ROS_INFO("Adding DMP with name %s and execution duration %f.", request.dmps_model[i].name.c_str(), request.execution_duration[i]);

	if (!dmp_struct.dmp->initializeFromDisc(request.directory_name, request.dmps_model[i].name)) {
	    ROS_ERROR("Could not set DMP with name id %s from disc.", request.dmps_model[i].name.c_str());
	    response.info.assign(std::string("Could not set DMP with name id ") + request.dmps_model[i].name + std::string("."));
	    response.return_code = dmp_controller::AddToExecuteDMPQueue::Response::SERVICE_CALL_FAILED;
	    return false;
	}

	Eigen::VectorXd goal(request.goal[i].state.size());
	for (int j = 0; j < static_cast<int>(request.goal[i].state.size()); j++)
	    goal(j) = request.goal[i].state[j];
        if (!dmp_struct.dmp->setup(goal)) {
	    ROS_ERROR("Could not setup the goal state in DMP with name id %s from disc.", request.dmps_model[i].name.c_str());
	    response.info.assign(std::string("Could not setup the goal of DMP with name if ") + request.dmps_model[i].name + std::string("."));
	    response.return_code = dmp_controller::AddToExecuteDMPQueue::Response::SERVICE_CALL_FAILED;
	    return false;
	}

	dmp_struct.execution_time = request.execution_duration[i];
	dmp_struct.dmp->changeExecutionTime(request.execution_duration[i]);

	dmp_structs.push_back(dmp_struct);
    }

    return true;
}


void DMPController::addToQueue(std::vector<DMPStruct> dmp_structs)
{
    std::vector<DMPStruct>::iterator dmp_itr;
    if (pthread_mutex_lock(&dmp_cmd_lock_) == 0) {
	for (dmp_itr = dmp_structs.begin(); dmp_itr != dmp_structs.end(); ++dmp_itr) {
	    run_buffer_.push_back(*dmp_itr);
	}
	pthread_mutex_unlock(&dmp_cmd_lock_);
    }
}



} //@namespace dmp_controller
