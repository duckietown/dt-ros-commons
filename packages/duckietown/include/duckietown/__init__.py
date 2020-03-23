"""

The `duckietown` library provides the parent class for all Duckietown ROS Nodes: :py:class:`DTROS`,
as well as customized ROS Publisher (:py:class:`DTPublisher`) and ROS Subscriber (:py:class:`DTSubscriber`) classes.
These classes extend the original ROS classes by adding an ``active`` property that can set the subscriber
or publisher on or off in a smart way (reducing significantly processing and network overhead).


.. autoclass:: duckietown.DTROS

.. autoclass:: duckietown.DTPublisher

.. autoclass:: duckietown.DTSubscriber


"""

#TODO: we don't want to decorate rospy when anything inside duckietown.* is imported, but
# only when duckietown.dtros.* is imported. This means that DTROS cannot be exposed here.


from .dtros import DTROS, TopicType, ModuleType, ParamType
