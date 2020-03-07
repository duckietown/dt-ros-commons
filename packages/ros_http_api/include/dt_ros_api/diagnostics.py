from duckietown import DTROS
from duckietown import TopicType

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


ROS_DIAGNOSTICS_TOPIC = "/watchtower01/camera_node/image/compressed"
ROS_DIAGNOSTICS_MSG_TYPE = CompressedImage


class ROSDiagnosticsListener(DTROS):

    def __init__(self):
        super(ROSDiagnosticsListener, self).__init__('ROS_HTTP_API')

        self.sub = self.subscriber(
            ROS_DIAGNOSTICS_TOPIC,
            ROS_DIAGNOSTICS_MSG_TYPE,
            self._ros_diagnostics_data_cb,
            dt_healthy_hz=30,
            dt_topic_type=[TopicType.PERCEPTION]
        )

        self.pub = self.publisher(
            '/chatter',
            String,
            queue_size=1,
            dt_healthy_hz=0,
            dt_topic_type=[TopicType.GENERIC]
        )

        import rospy
        rospy.spin()

    def _ros_diagnostics_data_cb(self, data):
        pass
