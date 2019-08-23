import rospy


class DTROS(object):
    """Parent class for all Duckietown ROS nodes

    All Duckietown ROS nodes should inherit this class. This class provides
    some basic common functionality that most of the ROS nodes need. By keeping
    these arguments and methods in a parent class, we can ensure consistent and
    reliable behaviour of all ROS nodes in the Duckietown universe.

    In particular, the DTROS class provides:
    - Logging: DTROS provides the `log` method as a wrapper around the ROS logging
        services. It will automatically append the ROS node name to the message.
    - Parameters handling:  DTROS provides the `parameters` and `parametersChanged` attributes
        and automatically updates them if it detects a change in the Parameter Server.
    - Shutdown procedure: a common shutdown procedure for ROS nodes. Should be attached
        via `rospy.on_shutdown(nodeobject.onShutdown)`.

    Every children node should call the initializer of `DTROS`. This should be done
    by having the following line at the top of the children node `__init__` method. The
    DTROS initializer will:

    - Setup the `node_name` attribute to the node name passed by ROS (using `rospy.get_name()`)
    - Initialize an empty `parameters` dictionary where all configurable ROS parameters should
        be stored. A boolean attribute `parametersChanged` is also initialized. This will be set to
        `True` when the ... method detects a change in a parameter value in the
        `ROS Parameter Server <https://wiki.ros.org/Parameter%20Server>`_ and changes the value
        of at least one parameter.
    - Start a recurrent timer that calls `updateParameters` regularly to
        check if any parameter has been updated

    Args:
        parameters_update_period (floar): how often to check for new parameters (in seconds)

    Attributes:
        node_name (str): the name of the node
        parameters (dict): a dictionary that holds pairs `('~param_name`: param_value)`. Note that
            parameters should be given in private namespace (starting with `~`)
        parametersChanged (bool): a boolean indicator if the
        is_shutdown (bool): will be set to `True` when the `onShutdown` method is called

    """

    def __init__(self, parameters_update_period=1.0):

        # Initialize
        self.node_name = rospy.get_name()
        self.log("Initializing...")
        self.is_shutdown = False

        # Initialize parameters handling
        self.parameters = dict()
        self.parametersChanged = True
        self.__updateParametersTimer = rospy.Timer(period=parameters_update_period,
                                                   callback=self.updateParameters(),
                                                   oneshot=False)


    def log(self, msg, type='info'):
        """ Passes a logging message to the ROS logging methods.

        Attaches the ros name to the beginning of the message and passes it to
        a suitable ROS logging method. Use the `type` argument to select the method
        to be used (`debug` for `rospy.logdebug`,
        `info` for `rospy.loginfo`, `warn` for `rospy.logwarn`,
        `err` for `rospy.logerr`, `fatal` for `rospy.logfatal`).

        Args:
            msg (str): the message content
            type (str): one of `debug`, `info`, `warn`, `err`, `fatal`

        Raises:
            ValueError: if the `type` argument is not one of the supported types

        """

        full_msg = "[%s] %s" % (self.node_name, msg)

        if type=='debug':
            rospy.logdebug(full_msg)
        elif type=='info':
            rospy.loginfo(full_msg)
        elif type=='warn':
            rospy.logwarn(full_msg)
        elif type=='err':
            rospy.logerr(full_msg)
        elif type=='fatal':
            rospy.logfatal(full_msg)
        else:
            raise ValueError("Type argument value %s is not supported!" % type)

    def updateParameters(self, event):
        """Keeping the node parameters up to date with the parameter server.

        Goes through all the node parameters and check if the value for any of them is changed
        in the parameter server. If there is a parameter that wasn't found, it will throw an `KeyError`
        exception.

        If a value of a parameter is changed, it will be updated and `self.parametersChanged` will be set
        to `True` in order to inform other methods to adjust their behavior.

        Raises:
            KeyError: if one of the parameters is not found in the parameter server

        """

        for param_name in self.parameters:
            new_value = rospy.get_param(param_name)
            if new_value != self.parameters[param_name]:
                self.parameters[param_name] = new_value
                self.parametersChanged = True
                self.log("Setting parameter %s = %s " % (param_name, new_value))

    def onShutdown(self):
        """Shutdown procedure."""

        self.is_shutdown = True
        self.__updateParametersTimer.shutdown()
        self.log("Shutdown.")




