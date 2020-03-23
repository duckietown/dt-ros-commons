# Node health (this has to match duckietown.NodeHealth)
uint8 NODE_HEALTH_UNKNOWN = 0
uint8 NODE_HEALTH_STARTING = 1
uint8 NODE_HEALTH_HEALTHY = 2
uint8 NODE_HEALTH_WARNING = 3
uint8 NODE_HEALTH_ERROR = 4
uint8 NODE_HEALTH_FATAL = 5

Header header
string name             # Node publishing this message
uint8 health            # Node health (see NODE_HEALTH_X above)
string health_reason    # String describing the reason for this health status (if any)
float32 health_stamp    # Time when the health status changed into the current
bool enabled            # Status of the switch
string module_type      # Module containing this node
string module_instance  # ID of the instance of the module running this node
