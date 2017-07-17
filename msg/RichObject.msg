Header header
# ARuco ID of the object
uint32 id
# Last time these object properties were updated
time last_update
# Absolute position of the object
geometry_msgs/Point position
# Absolute orientation of the object
geometry_msgs/Quarternion orientation
# Relative distances to avatars
float32[] proximities
# Color category (0=R, 1=G, 2=B)
uint32 color
# List of possible owners, with 0 (unowned) always the first
uint32[] owners
# List of (possibly non-exclusive) ownership probabilities
float32[] ownership
# Indicates if object is avatar or landmark
bool is_avatar
bool is_landmark

