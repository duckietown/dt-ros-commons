ROS Package: duckietown_msgs
============================

.. contents::

The `duckietown_msgs` package contains all messages that are used for inter-node communication
via ROS in Duckietown.


Messages
--------

..
    TODO: Fix relative linking to literalinclude messages and then add full message list

Led
^^^

File: ``duckietown_msgs/AntiInstagramThresholds.msg``

.. code-block:: none
    std_msgs/ColorRGBA rgba
    float32 frequency

Led Pattern
^^^^^^^^^^^

File: ``duckietown_msgs/AntiInstagramThresholds.msg``

.. code-block:: none
    Header header
    uint8 priority
    float32 lifespan
    duckietown_msgs/Led[] leds
