import rospy
import time
import rosnode
from threading import Semaphore

from duckietown.dtros.utils import get_ros_handler

from .constants import FETCH_NEW_NODES_EVERY_SECS
from .Node import Node


class ParameterServerHelper:

    def __init__(self):
        rospy.init_node('parameter_server_helper', log_level=rospy.DEBUG)
        rospy.loginfo('Node parameter_server_helper is up and running!')
        self._nodes = {}
        self._nodes_lock = Semaphore(1)
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
        # start updating the list of nodes
        self._update_nodes_list_timer = rospy.Timer(
            period=rospy.Duration.from_sec(FETCH_NEW_NODES_EVERY_SECS),
            callback=self._update_nodes_list,
            oneshot=False
        )
        # spin right away
        self._update_nodes_list()

    def _update_nodes_list(self, *args, **kwargs):
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
