"""
The `duckietown.dtros` library provides the parent class for all Duckietown ROS Nodes: `DTROS`,
as well as customized ROS Publisher (`DTPublisher`) and ROS Subscriber (`DTSubscriber`) classes.
These classes extend the original ROS classes by adding an `active` property that can set the
subscriber or publisher on or off in a smart way (reducing significantly processing and network
overhead).


.. autoclass:: duckietown.DTROS

.. autoclass:: duckietown.DTPublisher

.. autoclass:: duckietown.DTSubscriber


"""

import rospy
rospy.__instance__ = None

# keep a copy to the original rospy objects
setattr(rospy, '__init_node__', rospy.init_node)
setattr(rospy, '__Publisher__', rospy.Publisher)
setattr(rospy, '__Subscriber__', rospy.Subscriber)


def get_instance():
    return rospy.__instance__


from .dtros import DTROS
from .dtparam import DTParam
from .dtreminder import DTReminder

from .constants import TopicType, ModuleType, ParamType, NodeHealth

# perform rospy decoration
from .decoration import rospy_decorate
rospy_decorate()
del rospy_decorate
