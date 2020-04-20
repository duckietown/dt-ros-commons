import rospy

from .constants import TopicDirection
from .dttopic import DTTopic
from .singleton import get_instance


class DTPublisher(DTTopic, rospy.__Publisher__):
    """ A wrapper around ``rospy.Publisher``.

    This class is exactly the same as the standard
    `rospy.Publisher <http://docs.ros.org/api/rospy/html/rospy.topics.Publisher-class.html>`_
    with the only difference of an :py:attr:`active` attribute being added. Whenever the :py:meth:`publish` method is used,
    an actual message will be send only if :py:attr:`active` is set to ``True``.

    Args:
       name (:obj:`str`): resource name of topic, e.g. 'laser'
       data_class (:obj:`ROS Message class`): message class for serialization
       subscriber_listener (:obj:`SubscribeListener`): listener for subscription events. May be ``None``
       tcp_nodelay (:obj:`bool`): If ``True``, sets ``TCP_NODELAY`` on the publisher's socket (disables Nagle algorithm).
          This results in lower latency publishing at the cost of efficiency.
       latch (:obj:`bool`) - If ``True``, the last message published is 'latched', meaning that any future subscribers
          will be sent that message immediately upon connection.
       headers (:obj:`dict`) - If not ``None``, a dictionary with additional header key-values being
          used for future connections.
       queue_size (:obj:`int`) - The queue size used for asynchronously publishing messages from different
          threads. A size of zero means an infinite queue, which can be dangerous. When the keyword is not
          being used or when ``None`` is passed all publishing will happen synchronously and a warning message
          will be printed.

    Attributes:
       All standard rospy.Publisher attributes
       active (:obj:`bool`): A flag that if set to ``True`` will allow publishing`. If set to ``False``, any calls
          to :py:meth:`publish` will not result in a message being sent. Can be directly assigned.

    Raises:
       ROSException: if parameters are invalid

    """

    def __init__(self, *args, **kwargs):
        ros_args = {k: v for k, v in kwargs.items() if not k.startswith('dt_')}
        # call super constructor
        DTTopic.__init__(self)
        rospy.__Publisher__.__init__(self, *args, **ros_args)
        # dt arguments
        self.active = True
        # parse dt arguments
        self._parse_dt_args(kwargs)
        # register dt topic
        if not self._dt_is_ghost:
            self._register_dt_topic(TopicDirection.OUTBOUND)
        # register publisher
        if get_instance() is not None:
            get_instance()._register_publisher(self)
        # TODO: register for new subscriptions and publish new links on event

    def switch_off(self):
        self.active = False

    def switch_on(self):
        self.active = True

    def anybody_listening(self):
        return self.get_num_connections() > 0

    def publish(self, *args, **kwargs):
        """ A wrapper around the ``rospy.Publisher.publish`` method.

        This method is exactly the same as the standard
        `rospy.Publisher.publish <http://docs.ros.org/api/rospy/html/rospy.topics.Publisher-class.html#publish>`_
        with the only difference that a message is actually published only if the ``active``
        attribute is set to ``True``

        """
        if self.active:
            self._tick_frequency()
            # call super publish
            super(DTPublisher, self).publish(*args, **kwargs)
