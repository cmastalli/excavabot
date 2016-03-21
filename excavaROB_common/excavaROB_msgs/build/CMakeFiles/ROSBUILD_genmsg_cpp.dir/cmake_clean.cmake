FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/excavaROB_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/excavaROB_msgs/PoseStamped.h"
  "../msg_gen/cpp/include/excavaROB_msgs/Pose.h"
  "../msg_gen/cpp/include/excavaROB_msgs/VelocityBase.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
