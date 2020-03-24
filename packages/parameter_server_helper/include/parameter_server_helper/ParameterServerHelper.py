import rospy
import time
import rosnode
from threading import Semaphore

from duckietown.dtros.utils import get_ros_handler, apply_namespace
from duckietown.dtros.constants import DIAGNOSTICS_ROS_PARAMETERS_TOPIC

from .constants import FETCH_NEW_NODES_EVERY_SECS
from .Node import Node

from duckietown_msgs.msg import \
    NodeParameter, \
    DiagnosticsRosParameterArray


class ParameterServerHelper:

    def __init__(self):
        rospy.init_node('parameter_server_helper', log_level=rospy.DEBUG)
        self._nodes = {}
        self._nodes_lock = Semaphore(1)
        self._parameters = set()
        self._parameters_lock = Semaphore(1)
        self._ros_handler = get_ros_handler()
        # try to fetch the ROS Handler
        ntrial = 3
        while ntrial > 0:
            if self._ros_handler:
                break
            rospy.logerror('No ROS Handler found. Retrying in 5 seconds.')
            ntrial -= 1
            if ntrial == 0:
                rospy.logerror('No ROS Handler found. Exiting.')
                exit(1)
            time.sleep(5)
        # get paramUpdate method object from the ROS Handler
        self._rh_paramUpdate = self._ros_handler.paramUpdate
        # decorate paramUpdate
        setattr(self._ros_handler, 'paramUpdate', self._paramUpdate)
        # subscribe to diagnostics messages
        self._diagnostics_params_sub = rospy.Subscriber(
            apply_namespace(DIAGNOSTICS_ROS_PARAMETERS_TOPIC, 1),
            DiagnosticsRosParameterArray,
            self._diagnostics_params_cb,
            queue_size=100
        )
        # start updating the list of nodes
        self._update_nodes_list_timer = rospy.Timer(
            period=rospy.Duration.from_sec(FETCH_NEW_NODES_EVERY_SECS),
            callback=self._update_nodes_list,
            oneshot=False
        )
        # ---
        rospy.loginfo('Node parameter_server_helper is up and running!')
        # spin right away
        self._update_nodes_list()

    def register_parameter(self, name):
        self._parameters_lock.acquire()
        try:
            if name not in self._parameters:
                # subscribe to parameters change
                rospy.get_master().target.subscribeParam(
                    rospy.names.get_caller_id(),
                    rospy.core.get_node_uri(),
                    name
                )
                self._parameters.add(name)
        finally:
            self._parameters_lock.release()

    def _diagnostics_params_cb(self, data):
        parameters = {}
        for param in data.params:
            if param.node not in parameters:
                parameters[param.node] = set()
            parameters[param.node].add(param.name)
        # create new nodes
        self._update_nodes_list(nodes=list(parameters.keys()))
        # update nodes parameters
        for node, params in parameters.items():
            self._nodes[node].update_parameters_list(params=params)

    def _update_nodes_list(self, *args, **kwargs):
        nodes = []
        if 'nodes' in kwargs:
            nodes = kwargs['nodes']
        else:
            # this is called by the Timer, ask the Master node for the list of nodes
            rospy.logdebug('Fetching new nodes')
            try:
                nodes = rosnode.get_node_names()
            except rosnode.ROSNodeIOException:
                return
        # keep only the new nodes
        self._nodes_lock.acquire()
        try:
            new_nodes = filter(lambda n: n not in self._nodes, nodes)
            old_nodes = filter(lambda n: n not in nodes, self._nodes)
            if len(new_nodes):
                rospy.loginfo('Found %d new nodes:\n\t%s' % (
                    len(new_nodes), '\n\t'.join(new_nodes)
                ))
            elif len(old_nodes):
                rospy.loginfo('Found %d old nodes. Removing them:\n\t%s' % (
                    len(new_nodes), '\n\t'.join(old_nodes)
                ))
            else:
                rospy.logdebug('Nodes list is up to date')
            # create node descriptors
            for node in new_nodes:
                self._nodes[node] = Node(node)
            # remove node descriptors
            for node in old_nodes:
                self._nodes[node].shutdown()
                del self._nodes[node]
        finally:
            # release lock
            self._nodes_lock.release()

    def _paramUpdate(self, *args, **kwargs):
        # call super method
        self._rh_paramUpdate(*args, **kwargs)
        # pipe this to the nodes
        if len(args) < 2:
            rospy.logdebug('Received invalid paramUpdate call from Master')
            return
        # get what changed
        param_name, param_value = args[:2]
        rospy.logdebug('Received paramUpdate("%s", %s)' % (param_name, str(param_value)))
        # get a copy of the nodes dict
        self._nodes_lock.acquire()
        try:
            # tell the nodes to check if they need to update
            for node in self._nodes.values():
                node.request_update(param_name)
        finally:
            self._nodes_lock.release()
