import inspect
import glob
import os
from pathlib import Path
from typing import Optional, List

import rospy
from copy import copy
import yaml

from std_srvs.srv import SetBool, SetBoolResponse
from duckietown_msgs.srv import (
    NodeGetParamsList,
    NodeGetParamsListResponse,
    NodeRequestParamsUpdate,
    NodeRequestParamsUpdateResponse,
)
from duckietown_msgs.msg import NodeParameter
from duckietown.dtros.constants import (
    NODE_GET_PARAM_SERVICE_NAME,
    NODE_REQUEST_PARAM_UPDATE_SERVICE_NAME,
    NODE_SWITCH_SERVICE_NAME,
)
from .dtparam import DTParam
from .constants import NodeHealth, NodeType
from .diagnostics import DTROSDiagnostics
from .dtpublisher import DTPublisher
from .dtsubscriber import DTSubscriber
from .utils import get_ros_handler
from .profiler import CodeProfiler


NODE_USER_CONFIG_LOCATION = "/data/config/nodes"
NODE_USER_GENERIC_CONFIG_NAME = "generic"

FSM_NODE_CONTROL: bool = os.environ.get("FSM_NODE_CONTROL", "0").lower() in ["1", "y", "yes"]


class DTROS(object):
    """
    Parent class for all Duckietown ROS nodes

    All Duckietown ROS nodes should inherit this class. This class provides
    some basic common functionality that most of the ROS nodes need. By keeping
    these arguments and methods in a parent class, we can ensure consistent and
    reliable behaviour of all ROS nodes in the Duckietown universe.

    In particular, the DTROS class provides:

    - Logging: By providing utility functions for logging such as `loginfo`, `logwarn`, etc.
    - Shutdown procedure: A common shutdown procedure for ROS nodes.
    - Switchable Subscribers and Publishers: :py:meth:`publisher` and :py:meth:`subscriber` return
      decorated subscribers and publishers that can be dynamically deactivated and reactivated.
    - Node deactivation and reactivation: through requesting ``False`` to the ``~switch``
      service all subscribers and publishers obtained through :py:meth:`publisher`
      and :py:meth:`subscriber` will be deactivated and the ``switch`` attribute will be set
      to ``False``. This switch can be
      used by computationally expensive parts of the node code that are not in callbacks in
      order to pause their execution.
    - We look for a robot specific parameter file and overwrite the default parameters
      if it exists

    Every children node should call the initializer of DTROS. This should be done
    by having the following line at the top of the children node ``__init__`` method::

        super(ChildrenNode, self).__init__(node_name='children_node_name')

    The DTROS initializer will:

    - Initialize the ROS node with name ``node_name``
    - Setup the ``node_name`` attribute to the node name passed by ROS (using ``rospy.get_name()``)
    - Add a ``rospy.on_shutdown`` hook to the node's :py:meth:`onShutdown` method
    - Setup a ``~switch`` service that can be used to deactivate and reactivate the node

    Args:
       node_name (:obj:`str`): a unique, descriptive name for the ROS node
       node_type (:py:class:`duckietown.dtros.NodeType`): a node type
       help (:obj: `str`): a node description
       dt_ghost (:obj: `bool`): (Internal use only) excludes the node from the diagnostics

    Attributes:
        node_name (:obj:`str`): the name of the node
        node_help (:obj:`str`): the description of the node
        node_type (:py:class:`duckietown.dtros.NodeType`): the node type
        is_shutdown (:obj:`bool`): whether the node is shutdown

    Properties:
        is_ghost:   (:obj:`bool`): (Internal use only) whether the node is a ghost
        switch:     (:obj:`bool`): current state of the switch (`true=ON`, `false=OFF`)
        parameters: (:obj:`list`): list of parameters defined within the node
        subscribers: (:obj:`list`): list of subscribers defined within the node
        publishers: (:obj:`list`): list of publishers defined within the node

    Service:
        ~switch:
            Switches the node between active state and inactive state.

            input:
                data (`bool`): The desired state. ``True`` for active, ``False`` for inactive.

            outputs:
                success (`bool`): ``True`` if the call succeeded
                message (`str`): Used to give details about success

    """

    def __init__(
        self,
        node_name,
        # DT parameters from here
        node_type,
        pkg_name=None,
        help=None,
        dt_ghost=False,
        fsm_controlled: bool = False,
    ):
        # configure singleton
        if rospy.__instance__ is not None:
            raise RuntimeError("You cannot instantiate two objects of type DTROS")
        rospy.__instance__ = self
        if not isinstance(node_type, NodeType):
            raise ValueError(
                "DTROS 'node_type' parameter must be of type 'duckietown.NodeType', "
                "got %s instead." % str(type(node_type))
            )

        # Initialize the node
        log_level = rospy.INFO
        if os.environ.get("DEBUG", 0) in ["1", "true", "True", "enabled", "Enabled", "on", "On"]:
            log_level = rospy.DEBUG
        rospy.init_node(node_name, log_level=log_level, __dtros__=True)
        self.node_name = rospy.get_name()
        self.node_help = help
        self.node_type = node_type
        self.log("Initializing...")
        self.is_shutdown = False
        self._is_ghost = dt_ghost
        self._health = NodeHealth.STARTING
        self._health_reason = None
        self._ros_handler = get_ros_handler()

        veh_name = self.node_name.split("/")[1]
        robot_param_file = f"{self.package_path}/config/{node_name}/{veh_name}.yaml"
        if os.path.isfile(robot_param_file):
            rospy.loginfo(f"[{self.package_name}] found robot specific parameter file.. loading")
            try:
                with open(robot_param_file, "r") as stream:
                    new_params = yaml.load(stream, Loader=yaml.Loader)
            except yaml.YAMLError:
                msg = f"[{self.package_name}] Error in parsing calibration file {robot_param_file}.. skipping"
                rospy.logerr(msg)
                rospy.signal_shutdown(msg)
            for key in new_params:
                rospy.set_param(f"{node_name}/{key}", new_params[key])

        # Initialize parameters handling
        self._parameters = dict()
        self._rh_paramUpdate = None
        if self._ros_handler is not None:
            # decorate the XMLRPC paramUpdate function
            self._rh_paramUpdate = self._ros_handler.paramUpdate
            setattr(self._ros_handler, "paramUpdate", self._param_update)

        # Handle publishers, subscribers, and the state switch
        self._switch: bool = False if (fsm_controlled and FSM_NODE_CONTROL) else True
        self.loginfo(f"Node starting with switch={self._switch}")
        self.logdebug(f"Switch configuration: "
                      f"fsm_controlled:{fsm_controlled}, env[FSM_NODE_CONTROL]:{FSM_NODE_CONTROL}")

        self._subscribers = list()
        self._publishers = list()

        # Load user configuration files
        self._load_user_parameter_files()

        # create switch service for node
        self.srv_switch = rospy.Service("~%s" % NODE_SWITCH_SERVICE_NAME, SetBool, self._srv_switch)
        # create services to manage parameters
        self._srv_get_params = rospy.Service(
            "~%s" % NODE_GET_PARAM_SERVICE_NAME, NodeGetParamsList, self._srv_get_params_list
        )
        self._srv_request_params_update = rospy.Service(
            "~%s" % NODE_REQUEST_PARAM_UPDATE_SERVICE_NAME,
            NodeRequestParamsUpdate,
            self._srv_request_param_update,
        )

        # register node against the diagnostics manager
        if DTROSDiagnostics.enabled():
            DTROSDiagnostics.getInstance().register_node(
                self.node_name, self.node_help, self.node_type, health=self._health
            )

        # provide a public interface to the context manager to use as `with self.profiler("PHASE")`
        self.profiler = CodeProfiler()

        # mark node as healthy and STARTED
        self.set_health(NodeHealth.STARTED)

        # register shutdown callback
        rospy.on_shutdown(self._on_shutdown)

    def _load_user_parameter_files(self):
        # load user configurations
        # - node-agnostic parameters
        generic_cfg_dir = os.path.join(NODE_USER_CONFIG_LOCATION, NODE_USER_GENERIC_CONFIG_NAME)
        self._load_user_parameter_files_from(generic_cfg_dir)
        # - node-specific parameters
        node_name = self.node_name.split("/", maxsplit=2)[-1]
        node_cfg_dir = os.path.join(NODE_USER_CONFIG_LOCATION, node_name)
        self._load_user_parameter_files_from(node_cfg_dir)

    def _load_user_parameter_files_from(self, path: str):
        user_cfg_star = os.path.join(path, "*.yaml")
        user_cfg_files = sorted(glob.glob(user_cfg_star))
        # accumulate configs here
        user_cfg = {}
        # load configuration files
        i = 1
        t = len(user_cfg_files)
        self.loginfo(f"Found {t} user configuration files in '{path}'")
        for user_cfg_file in user_cfg_files:
            self.loginfo(f"Loading user configuration file {i}/{t}: {user_cfg_file}")
            try:
                with open(user_cfg_file, "rt") as fin:
                    user_cfg.update(yaml.safe_load(fin))
            except Exception as e:
                self.logerr(f"Skipping user configuration file '{user_cfg_file}' because we "
                            f"failed loading it. The error reads: {str(e)}")
            i += 1
        # update ros parameters
        for k, v in user_cfg.items():
            k = f"~{k}"
            self.logdebug(f"User configuration files update parameter '{k}': "
                          f"{rospy.get_param(k, None)} -> {v} ")
            rospy.set_param(k, v)

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
    def subscribers(self) -> List[DTSubscriber]:
        """A list of all the subscribers of the node"""
        return self._subscribers

    @property
    def publishers(self) -> List[DTPublisher]:
        """A list of all the publishers of the node"""
        return self._publishers

    @property
    def package_path(self) -> Optional[str]:
        """The path to the catkin package this node belongs to"""
        # get all active locations where ROS packages are stored
        ROS_PACKAGE_PATH = os.environ.get("ROS_PACKAGE_PATH", "")
        package_paths = ROS_PACKAGE_PATH.split(":")
        # get the path to the file containing the child class (whoever is inheriting from this class)
        try:
            child_class_path: str = os.path.realpath(inspect.getfile(self.__class__))
        except Exception as e:
            self.logwarn(f"Could not determine the file hosting the subclass of DTROS. Error: {str(e)}")
            return None
        # match the subclass path with the places where packages are stored
        for package_path in package_paths:
            package_path = os.path.realpath(package_path)
            if child_class_path.startswith(package_path):
                # we have a match
                return package_path
        return None

    @property
    def package_name(self) -> Optional[str]:
        """The name of the catkin package this node belongs to"""
        package_path = self.package_path
        if not package_path:
            return None
        # the name of the package is simply the name of the directory containing it
        return Path(package_path).stem

    def set_health(self, health, reason=None):
        if not isinstance(health, NodeHealth):
            raise ValueError(
                "Argument 'health' must be of type duckietown.NodeHealth. "
                "Got %s instead" % str(type(health))
            )
        self.log("Health status changed [%s] -> [%s]" % (self._health.name, health.name))
        self._health = health
        self._health_reason = None if reason is None else str(reason)
        # update node health in the diagnostics manager
        if DTROSDiagnostics.enabled():
            DTROSDiagnostics.getInstance().update_node(health=self._health, health_reason=self._health_reason)

    def log(self, msg, type="info"):
        """Passes a logging message to the ROS logging methods.

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
        full_msg = "[%s] %s" % (self.node_name, msg)
        # pipe to the right logger
        if type == "debug":
            rospy.logdebug(full_msg)
        elif type == "info":
            rospy.loginfo(full_msg)
        elif type == "warn" or type == "warning":
            self.set_health(NodeHealth.WARNING, full_msg)
            rospy.logwarn(full_msg)
        elif type == "err" or type == "error":
            self.set_health(NodeHealth.ERROR, full_msg)
            rospy.logerr(full_msg)
        elif type == "fatal":
            self.set_health(NodeHealth.FATAL, full_msg)
            rospy.logfatal(full_msg)
        else:
            raise ValueError("Type argument value %s is not supported!" % type)

    def loginfo(self, msg):
        self.log(msg, type="info")

    def logerr(self, msg):
        self.log(msg, type="err")

    def logfatal(self, msg):
        self.log(msg, type="fatal")

    def logwarn(self, msg):
        self.log(msg, type="warn")

    def logdebug(self, msg):
        self.log(msg, type="debug")

    def on_switch_on(self):
        pass

    def on_switch_off(self):
        pass

    def _srv_switch(self, request):
        """
        Args:
            request (:obj:`std_srvs.srv.SetBool`): The switch request from the ``~switch`` callback

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
        # tell the node about the switch
        on_switch_fcn = {False: self.on_switch_off, True: self.on_switch_on}[self._switch]
        on_switch_fcn()
        # update node switch in the diagnostics manager
        if DTROSDiagnostics.enabled():
            DTROSDiagnostics.getInstance().update_node(enabled=self._switch)
        # create a response to the service call
        msg = "Node switched from [%s] to [%s]" % ("on" if old_state else "off", "on" if new_state else "off")
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
                    node=rospy.get_name(), name=p.name, help=p.help, type=p.type.value, **p.options()
                )
                for p in self.parameters
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
            self.logdebug("Received invalid paramUpdate call from Master")
            return
        # get what changed
        _, param_name, param_value = args[:3]
        param_name = param_name.rstrip("/")
        self.logdebug('Received paramUpdate("%s", %s)' % (param_name, str(param_value)))
        # update parameter value
        if param_name in self._parameters:
            param = self._parameters[param_name]
            # BUG: If we remove this IF, we get periodic update events with the same value again and again
            #       this should be fixed at the root
            # do not trigger an update event if the value has not changed
            if param.value != param_value:
                param.set_value(param_value)
                self.loginfo('Parameter "%s" has now the value [%s]' % (
                    param_name, str(self._parameters[param_name].value)
                ))
        else:
            # check if the parameter is the child of a monitored parameter
            for p in self._parameters.values():
                if param_name.startswith(p.name):
                    p.force_update()
                    self.loginfo(f"Detected change in child of '{p.name}'. Parameter '{p.name}' "
                                 f"has now value [{str(self._parameters[p.name].value)}]")

    def _add_param(self, param):
        if not isinstance(param, DTParam):
            raise ValueError("Expected type duckietown.DTParam, got %s instead" % str(type(param)))
        self._parameters[param.name] = param

    def _has_param(self, param):
        return rospy.names.resolve_name(param) in self._parameters

    def _register_publisher(self, publisher):
        self._publishers.append(publisher)

    def _register_subscriber(self, subscriber):
        self._subscribers.append(subscriber)

    def _on_shutdown(self):
        self.log("Received shutdown request.")
        self.is_shutdown = True
        # call node on_shutdown
        self.on_shutdown()

    def on_shutdown(self):
        # this function does not do anything, it is called when the node shuts down.
        # It can be redefined by the user in the final node class.
        pass
