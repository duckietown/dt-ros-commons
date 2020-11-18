#!/usr/bin/env python3
import threading

import rospy

from duckietown.dtros import DTROS, NodeType
from dt_communication_utils import DTCommunicationGroup, ANYBODY_BUT_ME

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class TFBridgeNode(DTROS):

    PUBLISH_TF_STATIC_EVERY_SECS = 10

    def __init__(self):
        super(TFBridgeNode, self).__init__(
            node_name='tf_bridge_node',
            node_type=NodeType.SWARM
        )
        # get static parameter - ~veh
        try:
            self.robot_hostname = rospy.get_param('~veh')
        except KeyError:
            self.logerr('The parameter ~veh was not set, the node will abort.')
            exit(1)
        # get static parameter - ~robot_type
        try:
            self.robot_type = rospy.get_param('~robot_type')
        except KeyError:
            self.logerr('The parameter ~robot_type was not set, the node will abort.')
            exit(2)
        # create communication group
        self._group = DTCommunicationGroup("/tf", TransformStamped)
        # create static tfs holder and access semaphore
        self._static_tfs = {}
        self._static_tfs_sem = threading.Semaphore(1)
        # create publishers
        self._tf_pub = self._group.Publisher()
        self._tf_static_timer = rospy.Timer(
            rospy.Duration(self.PUBLISH_TF_STATIC_EVERY_SECS),
            self._publish_static_tfs
        )
        # create local subscribers
        self._local_tf_sub = rospy.Subscriber(
            '/tf',
            TFMessage,
            self._cb_local_tf,
            queue_size=20
        )
        self._local_tf_static_sub = rospy.Subscriber(
            '/tf_static',
            TFMessage,
            self._cb_local_tf_static,
            queue_size=20
        )
        # create global subscriber
        self._tf_broadcaster = None
        self._tf_static_broadcaster = None
        if self.robot_type in ['duckietown']:
            self._global_tf_sub = self._group.Subscriber(self._cb_global_tf)
            self._tf_broadcaster = TransformBroadcaster()
            self._tf_static_broadcaster = StaticTransformBroadcaster()

    def on_shutdown(self):
        if self._group is not None:
            self._group.shutdown()

    def _cb_local_tf(self, msg):
        for transform in msg.transforms:
            if not self._can_publish(transform):
                continue
            # publish
            self._tf_pub.publish(transform, destination=ANYBODY_BUT_ME)

    def _cb_local_tf_static(self, msg):
        self._static_tfs_sem.acquire()
        try:
            for transform in msg.transforms:
                if not self._can_publish(transform):
                    continue
                key = (transform.header.frame_id, transform.child_frame_id)
                self._static_tfs[key] = transform
        finally:
            self._static_tfs_sem.release()

    def _publish_static_tfs(self, *_):
        tfs = []
        self._static_tfs_sem.acquire()
        try:
            for transform in self._static_tfs.values():
                tfs.append(transform)
        finally:
            self._static_tfs_sem.release()
        # publish
        for tf in tfs:
            self._tf_pub.publish(tf, destination=ANYBODY_BUT_ME)

    def _cb_global_tf(self, msg, header):
        # make sure we are not getting boomerang messages (this should hot happen)
        if header.i_sent_this():
            self.logwarn('Received boomerang messages. This should not have happened.')
            return
        # forward TF locally
        tf_bcaster = self._tf_broadcaster if msg.header.seq > 0 else self._tf_static_broadcaster
        if tf_bcaster:
            tf_bcaster.sendTransform(msg)

    def _can_publish(self, transform):
        if self.robot_type == 'duckietown':
            # `duckietown` devices can ONLY publish TFs with /world as parent
            if transform.header.frame_id.lstrip('/') != 'world':
                return False
        else:
            # other robots can ONLY publish what they have perceived themselves
            if not transform.header.frame_id.lstrip('/').startswith(f'{self.robot_hostname}/'):
                return False
        # ---
        return True


if __name__ == '__main__':
    node = TFBridgeNode()
    # spin forever
    rospy.spin()
