#
# A message for communicating information about currently visible objects (6x6x6cm cubes)
#

# We should timestamp this
std_msgs/Header header

# 
# The cubes
#

# Is this a green or red object
bool[] is_green

# The area in pixels the object occupies
uint32[] area

# The heading (relative to the robot's current pose) of a cube
float32[] heading

# The distance from the depth sensor to the cube
float32[] range
