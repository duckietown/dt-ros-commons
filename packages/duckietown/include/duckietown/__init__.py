"""
The `duckietown` library provides the parent class for all Duckietown ROS Nodes: `DTROS`,
as well as customized ROS Publisher (`DTPublisher`) and ROS Subscriber (`DTSubscriber`) classes.
These classes extend the original ROS classes by adding an `active` property that can set the subscriber
or publisher on or off in a smart way (reducing significantly processing and network overhead).


.. autoclass:: duckietown.DTROS

.. autoclass:: duckietown.DTPublisher

.. autoclass:: duckietown.DTSubscriber


"""

from .dtros import DTROS
from .dtpublisher import DTPublisher
from .dtsubscriber import DTSubscriber
