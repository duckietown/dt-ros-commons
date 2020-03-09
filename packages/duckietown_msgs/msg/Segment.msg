# Enum constants for a set of pre-defined colors.
uint8 WHITE=0
uint8 YELLOW=1
uint8 RED=2
uint8 BLUE=3
uint8 GREEN=4
uint8 BLACK=5

# Holds the value of the desired color
uint8 color

# Coordinates of the starting and ending pixels of the segment
# normalized by the image dimensions
duckietown_msgs/Vector2D[2] pixels_normalized

# Normal of the line segment
duckietown_msgs/Vector2D normal

# Ground-projected end points
geometry_msgs/Point[2] points
