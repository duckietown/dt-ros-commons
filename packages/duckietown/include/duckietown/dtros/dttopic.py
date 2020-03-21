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
        self._dt_is_ghost = _arg(kwargs, 'dt_ghost', False)
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

    def set_healthy_freq(self, healthy_hz):
        if DTROSDiagnostics.enabled():
            DTROSDiagnostics.getInstance().update_topic(
                self.resolved_name,
                healthy_freq=healthy_hz
            )
        self._dt_healthy_freq = healthy_hz

    def get_frequency(self):
        # get frequency from the diagnostics manager
        if DTROSDiagnostics.enabled():
            return DTROSDiagnostics.getInstance().get_topic_frequency(self.resolved_name)
        return -1

    def get_bandwidth(self):
        # get bandwidth from the diagnostics manager
        if DTROSDiagnostics.enabled():
            return DTROSDiagnostics.getInstance().get_topic_bandwidth(self.resolved_name)
        return -1

    def shutdown(self):
        topic_name = self.resolved_name
        self.unregister()
        # unregister topic from diagnostics manager
        if DTROSDiagnostics.enabled():
            DTROSDiagnostics.getInstance().unregister_topic(topic_name)


def _arg(kwargs, key, default):
    return kwargs[key] if key in kwargs else default
