ROS Package: duckietown_msgs
============================

.. contents::

The `duckietown_msgs` package contains all messages that are used for inter-node communication
via ROS in Duckietown.  Each of the Duckietown messages can be found documented below and live in the `dt-ros-commons/packages/duckietown_msgs` directory.


Diagnostics
-----------

DiagnosticsCodeProfiling
^^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/DiagnosticsCodeProfiling.msg``

**Raw Message Definition:**

.. code-block:: text

    string node                             # Node publishing this message
    string block                            # Name of the profiled code block
    float32 frequency                       # Execution frequency of the block
    float32 duration                        # Last execution time of the block (in seconds)
    string filename                         # Filename in which this block resides
    uint16[2] line_nums                     # Start and end line of the block in the file
    float32 time_since_last_execution       # Seconds since last execution

DiagnosticsCodeProfilingArray
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/DiagnosticsCodeProfilingArray.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    duckietown_msgs/DiagnosticsCodeProfiling[] blocks

DiagnosticsRosLink
^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/DiagnosticsRosLink.msg``

**Raw Message Definition:**

.. code-block:: text

    # Link direction
    uint8 LINK_DIRECTION_INBOUND = 0
    uint8 LINK_DIRECTION_OUTBOUND = 1

    string node         # Node publishing this message
    string topic        # Topic transferred over the link
    string remote       # Remote end of this link
    uint8 direction     # Link direction
    bool connected      # Status of the link
    string transport    # Type of transport used for this link
    uint64 messages     # Number of messages transferred over this link
    uint64 dropped      # Number of messages dropped over this link
    float32 bytes       # Volume of data transferred over this link
    float32 frequency   # Link frequency (Hz)
    float32 bandwidth   # Link bandwidth (byte/s)

DiagnosticsRosLinkArray
^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/DiagnosticsRosLinkArray.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    duckietown_msgs/DiagnosticsRosLink[] links

DiagnosticsRosNode
^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/DiagnosticsRosNode.msg``

**Raw Message Definition:**

.. code-block:: text

    # Node type (this has to match duckietown.NodeType)
    uint8 NODE_TYPE_GENERIC = 0
    uint8 NODE_TYPE_DRIVER = 1
    uint8 NODE_TYPE_PERCEPTION = 2
    uint8 NODE_TYPE_CONTROL = 3
    uint8 NODE_TYPE_PLANNING = 4
    uint8 NODE_TYPE_LOCALIZATION = 5
    uint8 NODE_TYPE_MAPPING = 6
    uint8 NODE_TYPE_SWARM = 7
    uint8 NODE_TYPE_BEHAVIOR = 8
    uint8 NODE_TYPE_VISUALIZATION = 9
    uint8 NODE_TYPE_INFRASTRUCTURE = 10
    uint8 NODE_TYPE_COMMUNICATION = 11
    uint8 NODE_TYPE_DIAGNOSTICS = 12
    uint8 NODE_TYPE_DEBUG = 20

    # Node health (this has to match duckietown.NodeHealth)
    uint8 NODE_HEALTH_UNKNOWN = 0
    uint8 NODE_HEALTH_STARTING = 5
    uint8 NODE_HEALTH_STARTED = 6
    uint8 NODE_HEALTH_HEALTHY = 10
    uint8 NODE_HEALTH_WARNING = 20
    uint8 NODE_HEALTH_ERROR = 30
    uint8 NODE_HEALTH_FATAL = 40

    Header header
    string name             # Node publishing this message
    string help             # Node description
    uint8 type              # Node type (see NODE_TYPE_X above)
    uint8 health            # Node health (see NODE_HEALTH_X above)
    string health_reason    # String describing the reason for this health status (if any)
    float32 health_stamp    # Time when the health status changed into the current
    bool enabled            # Status of the switch
    string uri              # RPC URI of the node
    string machine          # Machine hostname or IP where this node is running
    string module_type      # Module containing this node
    string module_instance  # ID of the instance of the module running this node

DiagnosticsRosParameterArray
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/DiagnosticsRosParameterArray.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    duckietown_msgs/NodeParameter[] params  # List of parameters

DiagnosticsRosProfiling
^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/DiagnosticsRosProfiling.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    duckietown_msgs/DiagnosticsRosProfilingUnit[] units  # List of profiling units

DiagnosticsRosProfilingUnit
^^^^^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/DiagnosticsRosProfilingUnit.msg``

**Raw Message Definition:**

.. code-block:: text

    # Link direction
    uint8 LINK_DIRECTION_INBOUND = 0
    uint8 LINK_DIRECTION_OUTBOUND = 1

    string node         # Node publishing this message
    string name         # Name of the profiled unit
    float32 time        # Execution time of the unit

DiagnosticsRosTopic
^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/DiagnosticsRosTopic.msg``

**Raw Message Definition:**

.. code-block:: text

    # Topic direction (this has to match duckietown.TopicDirection)
    uint8 TOPIC_DIRECTION_INBOUND = 0
    uint8 TOPIC_DIRECTION_OUTBOUND = 1

    # Topic type (this has to match duckietown.TopicType)
    uint8 TOPIC_TYPE_GENERIC = 0
    uint8 TOPIC_TYPE_DRIVER = 1
    uint8 TOPIC_TYPE_PERCEPTION = 2
    uint8 TOPIC_TYPE_CONTROL = 3
    uint8 TOPIC_TYPE_PLANNING = 4
    uint8 TOPIC_TYPE_LOCALIZATION = 5
    uint8 TOPIC_TYPE_MAPPING = 6
    uint8 TOPIC_TYPE_SWARM = 7
    uint8 TOPIC_TYPE_BEHAVIOR = 8
    uint8 TOPIC_TYPE_VISUALIZATION = 9
    uint8 TOPIC_TYPE_INFRASTRUCTURE = 10
    uint8 TOPIC_TYPE_COMMUNICATION = 11
    uint8 TOPIC_TYPE_DIAGNOSTICS = 12
    uint8 TOPIC_TYPE_DEBUG = 20

    string node                     # Node publishing this message
    string name                     # Topic object of the diagnostics
    string help                     # Topic description
    uint8 type                      # Topic type
    uint8 direction                 # Topic direction
    float32 frequency               # Topic frequency (Hz)
    float32 effective_frequency     # Topic (effective) frequency (Hz)
    float32 healthy_frequency       # Frequency at which this topic can be considered healthy
    float32 bandwidth               # Topic bandwidth (byte/s)
    bool enabled                    # Topic switch

DiagnosticsRosTopicArray
^^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/DiagnosticsRosTopicArray.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    duckietown_msgs/DiagnosticsRosTopic[] topics


Drone
-----

DroneControl
^^^^^^^^^^^^

**File:** ``duckietown_msgs/DroneControl.msg``

**Raw Message Definition:**

.. code-block:: text

    #Roll Pitch Yaw(rate) Throttle Commands, simulating output from
    #remote control. Values range from 1000 to 2000
    #which corespond to values from 0% to 100%

    float32 roll
    float32 pitch
    float32 yaw
    float32 throttle

DroneMode
^^^^^^^^^

**File:** ``duckietown_msgs/DroneMode.msg``

**Raw Message Definition:**

.. code-block:: text

    # Power supply status constants
    uint8 DRONE_MODE_DISARMED = 0
    uint8 DRONE_MODE_ARMED = 1
    uint8 DRONE_MODE_FLYING = 2

    # The drone status  as reported. Values defined above
    uint8 drone_mode


Commands
--------

CarControl.msg
^^^^^^^^^^^^^^

DuckiebotLED.msg
^^^^^^^^^^^^^^^^

FSMState.msg
^^^^^^^^^^^^

LEDPattern.msg
^^^^^^^^^^^^^^

Twist2DStamped.msg
^^^^^^^^^^^^^^^^^^

WheelsCmd.msg
^^^^^^^^^^^^^

WheelsCmdDBV2Stamped.msg
^^^^^^^^^^^^^^^^^^^^^^^^

WheelsCmdStamped.msg
^^^^^^^^^^^^^^^^^^^^


Image Pipeline
--------------

AntiInstagramThresholds.msg
^^^^^^^^^^^^^^^^^^^^^^^^^^^

AprilTagDetection.msg
^^^^^^^^^^^^^^^^^^^^^

AprilTagDetectionArray.msg
^^^^^^^^^^^^^^^^^^^^^^^^^^

AprilTagsWithInfos.msg
^^^^^^^^^^^^^^^^^^^^^^

DisplayFragment.msg
^^^^^^^^^^^^^^^^^^^

LineFollowerStamped.msg
^^^^^^^^^^^^^^^^^^^^^^^

ObstacleImageDetection.msg
^^^^^^^^^^^^^^^^^^^^^^^^^^

ObstacleImageDetectionList.msg
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ObstacleProjectedDetection.msg
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ObstacleProjectedDetectionList.msg
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ObstacleType.msg
^^^^^^^^^^^^^^^^

Pixel.msg
^^^^^^^^^

Rect.msg
^^^^^^^^

Rects.msg
^^^^^^^^^

SceneSegments.msg
^^^^^^^^^^^^^^^^^

Segment.msg
^^^^^^^^^^^

SegmentList.msg
^^^^^^^^^^^^^^^

TagInfo.msg
^^^^^^^^^^^

StreetNameDetection.msg
^^^^^^^^^^^^^^^^^^^^^^^

VehicleCorners.msg
^^^^^^^^^^^^^^^^^^


Navigation
----------

CoordinationClearance.msg
^^^^^^^^^^^^^^^^^^^^^^^^^

CoordinationSignal.msg
^^^^^^^^^^^^^^^^^^^^^^

EpisodeStart.msg
^^^^^^^^^^^^^^^^

IntersectionPose.msg
^^^^^^^^^^^^^^^^^^^^

IntersectionPoseImg.msg
^^^^^^^^^^^^^^^^^^^^^^^

IntersectionPoseImgDebug.msg
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

KinematicsParameters.msg
^^^^^^^^^^^^^^^^^^^^^^^^

KinematicsWeights.msg
^^^^^^^^^^^^^^^^^^^^^

LanePose.msg
^^^^^^^^^^^^

LEDInterpreter.msg
^^^^^^^^^^^^^^^^^^

MaintenanceState.msg
^^^^^^^^^^^^^^^^^^^^

Pose2DStamped.msg
^^^^^^^^^^^^^^^^^

SignalsDetection.msg
^^^^^^^^^^^^^^^^^^^^

SignalsDetectionETHZ17.msg
^^^^^^^^^^^^^^^^^^^^^^^^^^

StopLineReading.msg
^^^^^^^^^^^^^^^^^^^

StreetNames.msg
^^^^^^^^^^^^^^^

ThetaDotSample.msg
^^^^^^^^^^^^^^^^^^

Trajectory.msg
^^^^^^^^^^^^^^

TurnIDandType.msg
^^^^^^^^^^^^^^^^^

VehiclePose.msg
^^^^^^^^^^^^^^^

Vsample.msg
^^^^^^^^^^^


Primitive
---------

BoolStamped.msg
^^^^^^^^^^^^^^^

Vector2D.msg
^^^^^^^^^^^^


ROS
---

NodeParameter.msg
^^^^^^^^^^^^^^^^^

ParamTuner.msg
^^^^^^^^^^^^^^

SourceTargetNodes.msg
^^^^^^^^^^^^^^^^^^^^^


Sensors
-------

ButtonEvent.msg
^^^^^^^^^^^^^^^

DuckieSensor.msg
^^^^^^^^^^^^^^^^

EncoderStamped.msg
^^^^^^^^^^^^^^^^^^

LEDDetection.msg
^^^^^^^^^^^^^^^^

LEDDetectionArray.msg
^^^^^^^^^^^^^^^^^^^^^

LEDDetectionDebugInfo.msg
^^^^^^^^^^^^^^^^^^^^^^^^^

LightSensor.msg
^^^^^^^^^^^^^^^

WheelEncoderStamped.msg
^^^^^^^^^^^^^^^^^^^^^^^

