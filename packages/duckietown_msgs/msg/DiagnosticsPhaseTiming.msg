# Message that carries timing information about a timed section of code
string node_node                # Node publishing this message
string phase_name               # Name of the particular phase
uint16 num_events               # Number of events recorded in the set period
float32 frequency               # Frequency of executing the phase
float32 avg_duration            # Average time (in sec) that the phase takes to execute
string filename                 # Filename in which this phase resides
uint16[2] line_nums             # Start and end line of the phase in the file
