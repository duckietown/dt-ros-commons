import time
import rospy

from .constants import TopicType, MIN_TOPIC_FREQUENCY_SUPPORTED, MAX_TOPIC_FREQUENCY_SUPPORTED
from .diagnostics import DTROSDiagnostics


class DTTopic(rospy.topics.Topic):
    """
    This is a generic DT Publisher/Subscriber.
    We called it Topic to follow the convention used by rospy.
    """
    def __init__(self, *args, **kwargs):
        self._dt_healthy_freq = -1
        self._dt_topic_type = TopicType.GENERIC
        self._dt_is_ghost = False

    def _parse_dt_args(self, kwargs):
        # parse dt arguments
        self._dt_healthy_freq = _arg(kwargs, 'dt_healthy_hz', -1)
        self._dt_topic_type = _arg(kwargs, 'dt_topic_type', TopicType.GENERIC)
        self._dt_is_ghost = _arg(kwargs, 'dt_ghost', False)
        # sanitize dt_type
        if not isinstance(self._dt_topic_type, TopicType):
            rospy.logerror('The type "{:s}" is not supported. '.format(str(self._dt_topic_type)) +
                           'An instance of duckietown.TopicType is expected')
            self._dt_topic_type = TopicType.GENERIC
        # topic statistics
        self._last_frequency_tick = -1
        self._frequency = 0.0

    def _register_dt_topic(self, direction):
        # register topic to diagnostics manager
        if DTROSDiagnostics.enabled():
            DTROSDiagnostics.getInstance().register_topic(
                self.resolved_name,
                direction,
                self._dt_healthy_freq,
                self._dt_topic_type,
                self
            )

    def set_healthy_freq(self, healthy_hz):
        self._dt_healthy_freq = healthy_hz
        if DTROSDiagnostics.enabled():
            DTROSDiagnostics.getInstance().update_topic(
                self.resolved_name,
                healthy_freq=self._dt_healthy_freq
            )

    def get_frequency(self):
        return self._frequency

    def get_bandwidth(self):
        # get bandwidth from the diagnostics manager
        if DTROSDiagnostics.enabled():
            return DTROSDiagnostics.getInstance().get_topic_bandwidth(self.resolved_name)
        return -1

    def _tick_frequency(self):
        if self._last_frequency_tick > 0:
            elapsed = time.time() - self._last_frequency_tick
            frequency = float(self._frequency * 0.3 + (1.0 / elapsed) * 0.7)
            # reject any reading above the max frequency, snap to 0 Hz once below min frequency
            if MIN_TOPIC_FREQUENCY_SUPPORTED <= frequency <= MAX_TOPIC_FREQUENCY_SUPPORTED:
                self._frequency = frequency
            else:
                self._frequency = 0.0
        self._last_frequency_tick = time.time()

    def shutdown(self):
        topic_name = self.resolved_name
        self.unregister()
        # unregister topic from diagnostics manager
        if DTROSDiagnostics.enabled():
            DTROSDiagnostics.getInstance().unregister_topic(topic_name)


def _arg(kwargs, key, default):
    return kwargs[key] if key in kwargs else default
