FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/excavaROB_mechanism_controllers/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/excavaROB_mechanism_controllers/msg/__init__.py"
  "../src/excavaROB_mechanism_controllers/msg/_BaseControllerState.py"
  "../src/excavaROB_mechanism_controllers/msg/_Odometer.py"
  "../src/excavaROB_mechanism_controllers/msg/_BaseOdometryState.py"
  "../src/excavaROB_mechanism_controllers/msg/_DebugInfo.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
