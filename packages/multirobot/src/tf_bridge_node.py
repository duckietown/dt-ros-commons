#!/usr/bin/env python3

import rospy

from duckietown.dtros import DTROS, NodeType
from dt_communication_utils import DTCommunicationGroup, ANYBODY_BUT_ME
from dt_device_utils import get_device_tag_id

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TFBridgeNode(DTROS):

    def __init__(self):
        super(TFBridgeNode, self).__init__(
            node_name='tf_bridge_node',
            node_type=NodeType.SWARM
        )
        # get static parameter - ~veh
        self.robot_hostname = rospy.get_param('~veh', None)
        if self.robot_hostname is None:
            self.logerr('The parameter ~veh was not set, the node will abort.')
            exit(1)
        # get static parameter - ~veh
        self.robot_type = rospy.get_param('~robot_type', None)
        if self.robot_type is None:
            self.logerr('The parameter ~robot_type was not set, the node will abort.')
            exit(2)
        # create communication group
        self._group = DTCommunicationGroup("/tf", TransformStamped)
        # create publishers
        self._tf_pub = self._group.Publisher()
        # create local subscribers
        self._local_tf_sub = rospy.Subscriber(
            '~tf',
            TFMessage,
            self._cb_local_tf,
            queue_size=1
        )
        # create global subscriber
        self._tf_broadcaster = None
        if self.robot_type in ['duckietown']:
            self._global_tf_sub = self._group.Subscriber(self._cb_global_tf)
            self._tf_broadcaster = TransformBroadcaster(queue_size=10)

    def on_shutdown(self):
        self._group.shutdown()

    def _cb_local_tf(self, msg):
        for transform in msg.transforms:
            publish = False
            # robots can only publish what they have perceived themselves
            if transform.header.frame_id.startswith(f'/{self.robot_hostname}'):
                publish = True
            # `duckietown` devices can ALSO publish TFs with /world as parent
            if self.robot_type in ['duckietown'] and transform.header.frame_id == '/world':
                publish = True
            # publish
            if publish:
                self._tf_pub.publish(transform, destination=ANYBODY_BUT_ME)

    def _cb_global_tf(self, msg, header):
        # make sure we are not getting boomerang messages (this should hot happen)
        if header.i_sent_this():
            self.logwarn('Received boomerang messages. This should not have happened.')
            return
        # forward TF locally
        if self._tf_broadcaster:
            self._tf_broadcaster.sendTransform(msg)


if __name__ == '__main__':
    node = TFBridgeNode()
    # spin forever
    rospy.spin()
