Header header

# Enum: regions
uint8 REGION_FULL=0
uint8 REGION_HEADER=1
uint8 REGION_BODY=2

# fragment ID and destination region
string id
uint8 region

# fragment content
sensor_msgs/Image data

# location on the display where to show the fragment
sensor_msgs/RegionOfInterest location

# Z index in the Z-buffer of the segment
uint8 z

# Time-to-Live in seconds of the fragment (-1 for infinite, do not abuse)
int8 ttl