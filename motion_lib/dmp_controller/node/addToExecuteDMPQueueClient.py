#!/usr/bin/env python

import roslib; roslib.load_manifest('dmp_controller')

import rospy
from dynamic_movement_primitives.msg import Model
from dmp_controller.srv import AddToExecuteDMPQueue, AddToExecuteDMPQueueRequest, AddToExecuteDMPQueueResponse
from dmp_controller.msg import State

class add_to_execute_dmp_queue():
    def __init__(self):
	rospy.wait_for_service('dmp_arm_controller/add_to_execute_dmp_queue')

	dmp_queue_client = rospy.ServiceProxy('dmp_arm_controller/add_to_execute_dmp_queue', AddToExecuteDMPQueue)

	self.request = AddToExecuteDMPQueueRequest()
	self.request.directory_name = '/home/cmastalli/ros_workspace/excavabot/motion_lib/dmp_learning/data_base/dmps/'

	# set dmp name
	dmp0 = Model()
	dmp1 = Model()
	dmp2 = Model()
#	dmp3 = Model()
	dmp0.name = 'startup'
	dmp1.name = 'left_excavation'
	dmp2.name = 'left_truck_loading'
#        dmp3.name = 'left_truck_loading_return'
	self.request.dmps_model.append(dmp0)
	self.request.dmps_model.append(dmp1)
	self.request.dmps_model.append(dmp2)
#	self.request.dmps_model.append(dmp3)

	# set goal state
	goal0 = State()
	goal1 = State()
	goal2 = State()
#	goal3 = State()
	goal0.state = [5.45, -0.40, 1.84, 0.0]
	goal1.state = [6.00, -0.50, 2.00, -0.30]
	goal2.state = [1.77, 4.20, 3.35, 1.35]
#        goal3.state = [5.84, -0.43, 1.33, 0.0]
	self.request.goal.append(goal0)
	self.request.goal.append(goal1)
	self.request.goal.append(goal2)
#	self.request.goal.append(goal3)

	# set execution time
	self.request.execution_duration.append(6)
	self.request.execution_duration.append(14)
	self.request.execution_duration.append(16)
#	self.request.execution_duration.append(16)

	print self.request

	try:
	    response = dmp_queue_client(self.request)
	    rospy.loginfo(response)
	except rospy.ServiceException, e:
	    print "Service call failed: %s"%e


if __name__ == "__main__":
    try:
	add_to_execute_dmp_queue()
    except rospy.ROSInterruptException:
	rospy.loginfo("Shutting down add to execute dmp queue node...")


