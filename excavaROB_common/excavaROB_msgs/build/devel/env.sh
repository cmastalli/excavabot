#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/cmastalli/ros_workspace/excavabot/excavaROB_common/excavaROB_msgs/build/devel', type 'exit' to leave"
  . "/home/cmastalli/ros_workspace/excavabot/excavaROB_common/excavaROB_msgs/build/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/cmastalli/ros_workspace/excavabot/excavaROB_common/excavaROB_msgs/build/devel'"
else
  . "/home/cmastalli/ros_workspace/excavabot/excavaROB_common/excavaROB_msgs/build/devel/setup.sh"
  exec "$@"
fi
