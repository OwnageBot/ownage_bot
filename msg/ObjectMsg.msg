# ARuco ID of the object
int32 id

# Human-readable name
string name

# Last time these object properties were updated
time t_last_update

# Absolute position of the object
geometry_msgs/Point position

# Absolute orientation of the object
geometry_msgs/Quaternion orientation

# Instantaneous speed of object
float32 speed

# Relative distances to avatars
float32[] proximities

# Color category
string color

# List of possible owners
int32[] owners

# List of (possibly non-exclusive) ownership probabilities
float32[] ownership

# List of category labels
string[] categories

# List of category membership probabilities
float32[] categoriness

# List of recent actors
int32[] actors

# List of last action times for each actor
time[] t_last_actions

# Indicates if object is an avatar
bool is_avatar

