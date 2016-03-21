FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/excavaROB_mechanism_controllers/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/excavaROB_mechanism_controllers/BaseControllerState.h"
  "../msg_gen/cpp/include/excavaROB_mechanism_controllers/Odometer.h"
  "../msg_gen/cpp/include/excavaROB_mechanism_controllers/BaseOdometryState.h"
  "../msg_gen/cpp/include/excavaROB_mechanism_controllers/DebugInfo.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
