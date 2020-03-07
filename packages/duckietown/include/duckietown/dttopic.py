import rospy
from types import ListType

from .constants import TopicType
from .diagnostics import DTROSDiagnostics


class DTTopic(rospy.topics.Topic):
    """
    This is a generic DT Publisher/Subscriber.
    We called it Topic to follow the convention used by rospy.
    """

    def _parse_dt_args(self, kwargs):
        # parse dt arguments
        self._dt_healthy_freq = _arg(kwargs, 'dt_healthy_hz', -1)
        self._dt_topic_type = _arg(kwargs, 'dt_topic_type', [])
        # sanitize dt_type
        if not isinstance(self._dt_topic_type, ListType):
            self._dt_topic_type = [self._dt_topic_type]
        for t in self._dt_topic_type:
            if not isinstance(t, TopicType):
                rospy.logerror('The type "{:s}" is not supported. '.format(str(t)) +
                               'An instance of duckietown.TopicType is expected')

    def _register_dt_topic(self, direction):
        # register topic to diagnostics manager
        if DTROSDiagnostics.enabled():
            DTROSDiagnostics.getInstance().register_topic(
                self.resolved_name,
                direction,
                self._dt_healthy_freq,
                self._dt_topic_type
            )


def _arg(kwargs, key, default):
    return kwargs[key] if key in kwargs else default
