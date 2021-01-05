Header header

# Enum: region
uint8 REGION_FULL=0
uint8 REGION_HEADER=1
uint8 REGION_BODY=2
uint8 REGION_FOOTER=3

# Enum: screen
uint8 SCREEN_ALL=255

# fragment ID and destination screen and region
string id
uint8 region
uint8 screen

# fragment content
sensor_msgs/Image data

# location on the display where to show the fragment
sensor_msgs/RegionOfInterest location

# Z index in the Z-buffer of the segment
uint8 z

# Time-to-Live in seconds of the fragment (-1 for infinite, do not abuse)
int8 ttl