uint8 PARAM_TYPE_UNKNOWN = 0
uint8 PARAM_TYPE_STRING = 1
uint8 PARAM_TYPE_INT = 2
uint8 PARAM_TYPE_FLOAT = 3
uint8 PARAM_TYPE_BOOL = 4

string node         # Name of the node
string name         # Name of the parameter (fully resolved)
uint8 type          # Type of the parameter (see PARAM_TYPE_X above)
float32 min_value   # Min value (for type INT, UINT, and FLOAT)
float32 max_value   # Max value (for type INT, UINT, and FLOAT)
