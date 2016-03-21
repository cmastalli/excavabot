FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/excavaROB_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/excavaROB_msgs/msg/__init__.py"
  "../src/excavaROB_msgs/msg/_PoseStamped.py"
  "../src/excavaROB_msgs/msg/_Pose.py"
  "../src/excavaROB_msgs/msg/_VelocityBase.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
