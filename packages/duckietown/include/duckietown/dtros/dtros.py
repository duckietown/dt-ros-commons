import os
import rospy
from copy import copy

from std_srvs.srv import SetBool, SetBoolResponse
from duckietown_msgs.srv import \
    NodeGetParamsList, \
    NodeGetParamsListResponse, \
    NodeRequestParamsUpdate, \
    NodeRequestParamsUpdateResponse
from duckietown_msgs.msg import \
    NodeParameter, \
    DiagnosticsPhaseTiming, \
    DiagnosticsPhaseTimingArray
from duckietown.dtros.constants import \
    NODE_GET_PARAM_SERVICE_NAME, \
    NODE_REQUEST_PARAM_UPDATE_SERVICE_NAME, \
    NODE_SWITCH_SERVICE_NAME
from .dtparam import DTParam
from .constants import NodeHealth, NodeType
from .diagnostics import DTROSDiagnostics
from .utils import get_ros_handler
from .phase_timing import PhaseTimer
from .constants import TopicType


class DTROS(object):
    """
    Parent class for all Duckietown ROS nodes

    All Duckietown ROS nodes should inherit this class. This class provides
    some basic common functionality that most of the ROS nodes need. By keeping
    these arguments and methods in a parent class, we can ensure consistent and
    reliable behaviour of all ROS nodes in the Duckietown universe.

    In particular, the DTROS class provides:

    - Logging: DTROS provides the :py:meth:`log` method as a wrapper around the ROS logging
      services. It will automatically append the ROS node name to the message.
    - Parameters handling:  DTROS provides the :py:attr:`parameters` and :py:attr:`parametersChanged` attributes
      and automatically updates them if it detects a change in the Parameter Server.
    - Shutdown procedure: a common shutdown procedure for ROS nodes. Should be attached
      via ``rospy.on_shutdown(nodeobject.onShutdown)``.
    - Switchable Subscribers and Publishers: :py:meth:`publisher` and :py:meth:`subscriber` return
      modified subscribers and publishers that can be dynamically deactivated and reactivated
      by requesting ``False`` or ``True`` to the ``~switch`` service respectively.
    - Node deactivation and reactivation: through requesting ``False`` to the ``~switch``
      service all subscribers and publishers obtained through :py:meth:`publisher` and :py:meth:`subscriber`
      will be deactivated and the ``switch`` attribute will be set to ``False``. This switch can be
      used by computationally expensive parts of the node code that are not in callbacks in order to
      to pause their execution.

    Every children node should call the initializer of DTROS. This should be done
    by having the following line at the top of the children node ``__init__`` method::

        super(ChildrenNode, self).__init__(node_name='children_node_name')

    The DTROS initializer will:

    - Initialize the ROS node with name ``node_name``
    - Setup the ``node_name`` attribute to the node name passed by ROS (using ``rospy.get_name()``)
    - Add a ``rospy.on_shutdown`` hook to the node's :py:meth:`onShutdown` method
    - Initialize an empty :py:attr:`parameters` dictionary where all configurable ROS parameters should
      be stored. A boolean attribute :py:attr:`parametersChanged` is also initialized. This will be set to
      ``True`` when the :py:meth:`updateParameters` callback detects a change in a parameter value in the
      `ROS Parameter Server <https://wiki.ros.org/Parameter%20Server>`_ and changes the value
      of at least one parameter.
      Additionally,
      the :py:meth:`cbParametersChanged` method will be called. This can be redefined by children classes
      in order to implement custom behavior when a change in the parameters is detected.

      Note:
          Implementing the :py:meth:`cbParametersChanged` callback will not
          automatically set :py:attr:`parametersChanged` to ``False``! If you rely on such behavior, you need to
          set it in your redefined callback.
    - Start a recurrent timer that calls :py:meth:`updateParameters` regularly to
      check if any parameter has been updated
    - Setup a ``~switch`` service that can be used to deactivate and reactivate the node

    Args:
       node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Attributes:
       node_name (:obj:`str`): the name of the node

    Service:
        ~switch:
            Switches the node between active state and inactive state.

            input:
                data (`bool`): The desired state. ``True`` for active, ``False`` for inactive.

            outputs:
                success (`bool`): ``True`` if the call succeeded
                message (`str`): Used to give details about success

    """

    def __init__(self,
                 node_name,
                 # DT parameters from here
                 node_type,
                 help=None,
                 dt_ghost=False):
        # configure singleton
        if rospy.__instance__ is not None:
            raise RuntimeError('You cannot instantiate two objects of type DTROS')
        rospy.__instance__ = self
        if not isinstance(node_type, NodeType):
            raise ValueError(
                "DTROS 'node_type' parameter must be of type 'duckietown.NodeType', "
                "got %s instead." % str(type(node_type))
            )
        # Initialize the node
        log_level = rospy.INFO
        if os.environ.get('DEBUG', 0) in ['1', 'true', 'True', 'enabled', 'Enabled', 'on', 'On']:
            log_level = rospy.DEBUG
        rospy.init_node(node_name, log_level=log_level, __dtros__=True)
        self.node_name = rospy.get_name()
        self.node_help = help
        self.node_type = node_type
        self.log('Initializing...')
        self.is_shutdown = False
        self._is_ghost = dt_ghost
        self._health = NodeHealth.STARTING
        self._health_reason = None
        self._ros_handler = get_ros_handler()

        # Initialize parameters handling
        self._parameters = dict()
        self._rh_paramUpdate = None
        if self._ros_handler is not None:
            # decorate the XMLRPC paramUpdate function
            self._rh_paramUpdate = self._ros_handler.paramUpdate
            setattr(self._ros_handler, 'paramUpdate', self._param_update)
        # Handle publishers, subscribers, and the state switch
        self._switch = True
        self._subscribers = list()
        self._publishers = list()
        # create switch service for node
        self.srv_switch = rospy.Service(
            "~%s" % NODE_SWITCH_SERVICE_NAME,
            SetBool, self._srv_switch
        )
        # create services to manage parameters
        self._srv_get_params = rospy.Service(
            "~%s" % NODE_GET_PARAM_SERVICE_NAME,
            NodeGetParamsList, self._srv_get_params_list
        )
        self._srv_request_params_update = rospy.Service(
            "~%s" % NODE_REQUEST_PARAM_UPDATE_SERVICE_NAME,
            NodeRequestParamsUpdate, self._srv_request_param_update
        )
        # register node against the diagnostics manager
        if DTROSDiagnostics.enabled():
            DTROSDiagnostics.getInstance().register_node(
                self.node_name,
                self.node_help,
                self.node_type,
                health=self._health
            )

        # Add a phase timer for profiling and a publisher topic
        self._pub_phase_timer = rospy.Publisher(
            "~diagnostics/phase_times",
            DiagnosticsPhaseTimingArray,
            queue_size=10,
            dt_topic_type=TopicType.DEBUG
        )
        self._pub_phase_timer.register_subscribers_changed_cb(self._cb_phase_timing_subs_update)
        self._phasetimeTimer = None
        # provides both a pointer to _phase_timer (for internal use)
        # and a public interface to the context manager `with self.timed_phase("PHASE")`
        self.timed_phase = self._phase_timer = PhaseTimer()

        # mark node as healthy and STARTED
        self.set_health(NodeHealth.STARTED)
        # register shutdown callback
        rospy.on_shutdown(self._on_shutdown)

    # Read-only properties for the private attributes
    @property
    def is_ghost(self):
        """Whether this is a ghost node (diagnostics will skip it)"""
        return self._is_ghost

    # Read-only properties for the private attributes
    @property
    def switch(self):
        """Current state of the node on/off switch"""
        return self._switch

    @property
    def parameters(self):
        """List of parameters"""
        return copy(list(self._parameters.values()))

    @property
    def subscribers(self):
        """A list of all the subscribers of the node"""
        return self._subscribers

    @property
    def publishers(self):
        """A list of all the publishers of the node"""
        return self._publishers

    def set_health(self, health, reason=None):
        if not isinstance(health, NodeHealth):
            raise ValueError('Argument \'health\' must be of type duckietown.NodeHealth. '
                             'Got %s instead' % str(type(health)))
        self.log('Health status changed [%s] -> [%s]' % (self._health.name, health.name))
        self._health = health
        self._health_reason = None if reason is None else str(reason)
        # update node health in the diagnostics manager
        if DTROSDiagnostics.enabled():
            DTROSDiagnostics.getInstance().update_node(
                health=self._health,
                health_reason=self._health_reason
            )

    def log(self, msg, type='info'):
        """ Passes a logging message to the ROS logging methods.

        Attaches the ros name to the beginning of the message and passes it to
        a suitable ROS logging method. Use the `type` argument to select the method
        to be used (``debug`` for ``rospy.logdebug``,
        ``info`` for ``rospy.loginfo``, ``warn`` for ``rospy.logwarn``,
        ``err`` for ``rospy.logerr``, ``fatal`` for ``rospy.logfatal``).

        Args:
            msg (`str`): the message content
            type (`str`): one of ``debug``, ``info``, ``warn``, ``err``, ``fatal``

        Raises:
            ValueError: if the ``type`` argument is not one of the supported types

        """
        full_msg = '[%s] %s' % (self.node_name, msg)
        # pipe to the right logger
        if type == 'debug':
            rospy.logdebug(full_msg)
        elif type == 'info':
            rospy.loginfo(full_msg)
        elif type == 'warn' or type == 'warning':
            self.set_health(NodeHealth.WARNING, full_msg)
            rospy.logwarn(full_msg)
        elif type == 'err' or type == 'error':
            self.set_health(NodeHealth.ERROR, full_msg)
            rospy.logerr(full_msg)
        elif type == 'fatal':
            self.set_health(NodeHealth.FATAL, full_msg)
            rospy.logfatal(full_msg)
        else:
            raise ValueError('Type argument value %s is not supported!' % type)

    def loginfo(self, msg):
        self.log(msg, type='info')

    def logerr(self, msg):
        self.log(msg, type='err')

    def logfatal(self, msg):
        self.log(msg, type='fatal')

    def logwarn(self, msg):
        self.log(msg, type='warn')

    def logdebug(self, msg):
        self.log(msg, type='debug')

    def _srv_switch(self, request):
        """
        Args:
            request (:obj:`std_srvs.srv.SetBool`): The switch request from the ``~switch`` callback.

        Returns:
            :obj:`std_srvs.srv.SetBoolResponse`: Response for successful feedback

        """
        old_state = self._switch
        self._switch = new_state = request.data
        # propagate switch change to publishers and subscribers
        for pub in self.publishers:
            pub.active = self._switch
        for sub in self.subscribers:
            sub.active = self._switch
        # update node switch in the diagnostics manager
        if DTROSDiagnostics.enabled():
            DTROSDiagnostics.getInstance().update_node(
                enabled=self._switch
            )
        # create a response to the service call
        msg = 'Node switched from [%s] to [%s]' % (
            'on' if old_state else 'off',
            'on' if new_state else 'off'
        )
        # print out the change in state
        self.log(msg)
        # reply to the service call
        response = SetBoolResponse()
        response.success = True
        response.message = msg
        return response

    def _srv_get_params_list(self, request):
        """
        Args:
            request (:obj:`duckietown_msgs.srv.NodeGetParamsList`): Service request message.

        Returns:
            :obj:`duckietown_msgs.srv.NodeGetParamsList`: Parameters list

        """
        return NodeGetParamsListResponse(
            parameters=[
                NodeParameter(
                    node=rospy.get_name(),
                    name=p.name,
                    help=p.help,
                    type=p.type.value,
                    **p.options()
                ) for p in self.parameters
            ]
        )

    def _srv_request_param_update(self, request):
        """
        Args:
            request (:obj:`duckietown_msgs.srv.NodeRequestParamsUpdate`): Service request message.

        Returns:
            :obj:`duckietown_msgs.srv.NodeRequestParamsUpdate`: Success feedback

        """
        try:
            self._parameters[request.parameter].force_update()
            return NodeRequestParamsUpdateResponse(success=True)
        except (KeyError, rospy.exceptions.ROSException):
            return NodeRequestParamsUpdateResponse(success=False)

    def _param_update(self, *args, **kwargs):
        # call super method
        if self._rh_paramUpdate is not None:
            self._rh_paramUpdate(*args, **kwargs)
        # check data
        if len(args) < 3:
            self.logdebug('Received invalid paramUpdate call from Master')
            return
        # get what changed
        _, param_name, param_value = args[:3]
        param_name = param_name.rstrip('/')
        self.logdebug('Received paramUpdate("%s", %s)' % (param_name, str(param_value)))
        # update parameter value
        if param_name in self._parameters:
            self._parameters[param_name].set_value(param_value)
            self.loginfo('Parameter "%s" has now the value [%s]' % (
                param_name, str(self._parameters[param_name].value)
            ))

    def _add_param(self, param):
        if not isinstance(param, DTParam):
            raise ValueError('Expected type duckietown.DTParam, got %s instead' % str(type(param)))
        self._parameters[param.name] = param

    def _has_param(self, param):
        return param in self._parameters

    def _register_publisher(self, publisher):
        self._publishers.append(publisher)

    def _register_subscriber(self, subscriber):
        self._subscribers.append(subscriber)

    def _publish_phase_timing(self, event=None):
        with self.timed_phase("Preparing phase timing message"):
            phase_timings = self._phase_timer.get_statistics()
            # create timing array
            array_msg = DiagnosticsPhaseTimingArray()
            array_msg.header.stamp = rospy.Time.now()
            for phase_name in sorted(phase_timings.keys()):
                single_phase_stats = phase_timings[phase_name]
                # create phase object
                single_phase_msg = DiagnosticsPhaseTiming()
                single_phase_msg.node = self.node_name
                single_phase_msg.name = phase_name
                single_phase_msg.num_events = single_phase_stats.num_events
                single_phase_msg.frequency = single_phase_stats.frequency
                single_phase_msg.avg_duration = single_phase_stats.avg_duration
                single_phase_msg.filename = single_phase_stats.filename
                single_phase_msg.line_nums = [
                    single_phase_stats.line_nums[0], single_phase_stats.line_nums[1]
                ]
                # add phase to array message
                array_msg.phases.append(single_phase_msg)
            # publish phase timing
            self._pub_phase_timer.publish(array_msg)

    def _cb_phase_timing_subs_update(self, publisher):
        if not publisher.anybody_listening():
            self.loginfo('Phase Timing Recording switched OFF')
            self._phase_timer.stop_recording()
            if self._phasetimeTimer is not None and not self._phasetimeTimer._shutdown:
                self._phasetimeTimer.shutdown()
        else:
            self.loginfo('Phase Timing Recording switched ON')
            self._phase_timer.start_recording()
            if self._phasetimeTimer is None or self._phasetimeTimer._shutdown:
                self._phasetimeTimer = rospy.Timer(period=rospy.Duration.from_sec(5),
                                                   callback=self._publish_phase_timing,
                                                   oneshot=False)

    def _on_shutdown(self):
        self.log('Received shutdown request.')
        self.is_shutdown = True
        # turn off phase timer
        if self._phasetimeTimer is not None and not self._phasetimeTimer._shutdown:
            self._phasetimeTimer.shutdown()
        # call node on_shutdown
        self.on_shutdown()

    def on_shutdown(self):
        # this function does not do anything, it is called when the node shuts down.
        # It can be redefined by the user in the final node class.
        pass
