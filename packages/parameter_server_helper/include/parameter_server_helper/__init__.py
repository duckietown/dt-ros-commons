"""
The `parameter_server_helper` package provides an helper for the ROS parameter server
that allow us to monitor changes in parameters values which in turn allows us to notify
all the nodes that are using those parameters affected by the change.

This is how it works:
- Nodes create parameters using the given DTParam class
- Under the hood, a Service `~/get_parameters_list` is made available on that node
- The helper monitors the list of nodes:
    - When a new node shows up in the list, its parameters list is retrieved
    - For each parameter:
        - If the helper has not subscribed to this param yet, it subscribes
    - A local mapping <node, parameter> is kept
- When a change in a parameter is notified by the Master node to the helper
    - The helper pings the nodes using that parameter on the service `~/request_parameters_update`
    - The nodes will update their cache


"""

from .ParameterServerHelper import ParameterServerHelper
