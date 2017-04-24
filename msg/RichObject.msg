Header header
# ARuco ID of the object
uint32 id
# Absolute position and orientation of the object
geometry_msgs/PoseWithCovariance pose
# Color category (0=R, 1=G, 2=B)
uint32 color
# Last time these object properties were updated
time last_update
# List of possible owners, with 0 (unowned) always the first
uint32[] owners
# List of (possibly non-exclusive) ownership probabilities
float32[] ownership
# Indicates if object is avatar or landmark
bool is_avatar
bool is_landmark

