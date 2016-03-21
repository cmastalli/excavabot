# This represents an base controller state with reference coordinate frame and timestamp.
Header header
# Wheel joints 
string[] joint_names
# Linear and angular measured, desired and error base velocities
excavaROB_msgs/VelocityBase velocity_measured
excavaROB_msgs/VelocityBase velocity_desired
excavaROB_msgs/VelocityBase velocity_error
# Control signals
float64 control_signal_right_wheel
float64 control_signal_left_wheel


