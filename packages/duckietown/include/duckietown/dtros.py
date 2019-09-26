import rospy
from std_srvs.srv import SetBool, SetBoolResponse

from .dtpublisher import DTPublisher
from .dtsubscriber import DTSubscriber

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
    - Switchable Subscribers and Publishers: `DTROS.publisher()` and `DTROS.subscriber()` returns
      modified subscribers and publishers that can be dynamically deactivated and reactivated
      by requesting `False` or `True` to the `~switch` service respectively.
    - Node deactivation and reactivation: through requesting `Falce` to the `~switch`
      service all subscribers and publishers obtained through `DTROS.publisher()` and `DTROS.subscriber()`
      will be deactivated and the `switch` attribute will be set to `False`. This switch can be
      used by computationally expensive parts of the node code that are not in callbacks in ordert to
      to pause their execution.

    Every children node should call the initializer of `DTROS`. This should be done
    by having the following line at the top of the children node `__init__` method::

        super(ChildrenNode, self).__init__(node_name='children_node_name')

    The DTROS initializer will:

    - Initialize the ROS node with name `node_name`
    - Setup the `node_name` attribute to the node name passed by ROS (using `rospy.get_name()`)
    - Add a `rospy.on_shutdown` hook to the node's `onShutdown` method
    - Initialize an empty `parameters` dictionary where all configurable ROS parameters should
      be stored. A boolean attribute `parametersChanged` is also initialized. This will be set to
      `True` when the `updateParameters` callback detects a change in a parameter value in the
      `ROS Parameter Server <https://wiki.ros.org/Parameter%20Server>`_ and changes the value
      of at least one parameter.
    - Start a recurrent timer that calls `updateParameters` regularly to
      check if any parameter has been updated
    - Setup a `~switch` service that can be used to deactivate and reactivate the node

    Args:
       node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use
       parameters_update_period (:obj:`float`): how often to check for new parameters (in seconds). If
          it is 0, it will not run checks at all

    Attributes:
       node_name (:obj:`str`): the name of the node
       parameters (:obj:`dict`): a dictionary that holds pairs `('~param_name`: param_value)`. Note that
          parameters should be given in private namespace (starting with `~`)
       parametersChanged (:obj:`bool`): a boolean indticator if the
       is_shutdown (:obj:`bool`): will be set to `True` when the `onShutdown` method is called
       switch (:obj:`bool`): flag determining whether the node is active or not. Read-only, controlled through
          the `~switch` service

    Service:
        ~switch:
            Switches the node between active state and inative state.

            input:
                data ('bool`): The desired state. `True` for active, `False` for inactive.

            outputs:
                success (`bool`): `True` if the call succeeded
                message (`str`): Used to give details about success

    """

    def __init__(self, node_name, parameters_update_period=1.0):

        # Initialize the node
        rospy.init_node(node_name, anonymous=False)
        self.node_name = rospy.get_name()
        self.log('Initializing...')
        self.is_shutdown = False
        rospy.on_shutdown(self.onShutdown)

        # Initialize parameters handling
        self.parameters = dict()
        self.parametersChanged = False
        self._parameters_update_period = parameters_update_period
        if self._parameters_update_period > 0:
            self._updateParametersTimer = rospy.Timer(period=rospy.Duration.from_sec(self._parameters_update_period),
                                                       callback=self.updateParameters,
                                                       oneshot=False)

        # Handle publishers, subscribers, and the state switch
        self._switch = True
        self._subscribers = list()
        self._publishers = list()
        self.srv_switch = rospy.Service("~switch",
                                        SetBool,
                                        self.srvSwitch)


    # Read-only properties for the private attributes
    @property
    def switch(self):
        """Current state of the node on/off switch"""
        return self._switch

    @property
    def parameters_update_period(self):
        """The parameters update period requiested in the node initializer"""
        return self._parameters_update_period

    @property
    def subscribers(self):
        """A list of all the subscribers of the node"""
        return self._subscribers

    @property
    def publishers(self):
        """A list of all the publishers of the node"""
        return self._publishers

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

        full_msg = '[%s] %s' % (self.node_name, msg)

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
            raise ValueError('Type argument value %s is not supported!' % type)

    def updateParameters(self, event=None, verbose=True):
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
                if verbose:
                    self.log('Setting parameter %s = %s ' % (param_name, new_value))

    def srvSwitch(self, request):
        """

        Args:
            request (:obj:`std_srvs.srv.SetBool`): The switch request from the `~switch` callback.

        Returns:
            :obj:`std_srvs.srv.SetBoolResponse`: Response for successful feedback

        """

        old_state = self._switch
        new_state = request.data

        self._switch = new_state
        for pub in self.publishers:
            pub.active = self._switch
        for sub in self.subscribers:
            sub.active = self._switch


        msg = 'Node switched from %s to %s' % ('on' if old_state else 'off',
                                               'on' if new_state else 'off')
        response = SetBoolResponse()
        response.success = True
        response.message = msg
        self.log(msg)

        return response

    def subscriber(self, *args, **kwargs):
        """
        Wrapper around `rospy.Subscriber`.

        Provides an :obj:`DTSubscriber` instance that can be used just as `rospy.Subscriber`, but
        with the possibility to be deactivated.

        Returns:
           DTSubscriber: the resulting subscriber instance

        """

        sub = DTSubscriber(*args, **kwargs)
        self._subscribers.append(sub)

        return sub


    def publisher(self, *args, **kwargs):
        """
        Wrapper around `rospy.Publisher`.

        Provides an :obj:`DTPublisher` instance that can be used just as `rospy.Publisher`, but
        with the possibility to be deactivated.

        Returns:
           DTPublisher: the resulting publisher instance

        """

        pub = DTPublisher(*args, **kwargs)
        self._publishers.append(pub)

        return pub



    def onShutdown(self):
        """Shutdown procedure."""

        self.is_shutdown = True
        if self._parameters_update_period > 0:
            self._updateParametersTimer.shutdown()
        self.log('Shutdown.')
