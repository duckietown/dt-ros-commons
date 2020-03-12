# Node health (this has to match duckietown.NodeHealth)
uint8 NODE_HEALTH_UNKNOWN = 0
uint8 NODE_HEALTH_STARTING = 1
uint8 NODE_HEALTH_HEALTHY = 2
uint8 NODE_HEALTH_WARNING = 3
uint8 NODE_HEALTH_ERROR = 4
uint8 NODE_HEALTH_FATAL = 5

string node             # Node publishing this message
uint8 health            # Node health (see NODE_HEALTH_X above)
bool enabled            # Status of the switch
