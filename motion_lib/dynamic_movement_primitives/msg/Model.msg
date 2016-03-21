# This represents the model of determined DMP.
bool initialized
bool is_learned
string name
int32 num_trans_sys
string[] var_name
Trajectory traj
CanonicalSystem can_sys
dynamic_movement_primitives/TransformationSystem[] trans_sys
