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

CarControl
^^^^^^^^^^

**File:** ``duckietown_msgs/CarControl.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    float32 speed
    float32 steering
    bool need_steering

DuckiebotLED
^^^^^^^^^^^^

**File:** ``duckietown_msgs/DuckiebotLED.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    std_msgs/ColorRGBA[] colors

FSMState
^^^^^^^^

**File:** ``duckietown_msgs/FSMState.msg``

**Raw Message Definition:**

.. code-block:: text

    # pseudo constants
    string LANE_FOLLOWING="LANE_FOLLOWING"
    string INTERSECTION_COORDINATION="INTERSECTION_COORDINATION"
    string INTERSECTION_CONTROL="INTERSECTION_CONTROL"
    string NORMAL_JOYSTICK_CONTROL="NORMAL_JOYSTICK_CONTROL"
    string SAFE_JOYSTICK_CONTROL="SAFE_JOYSTICK_CONTROL"
    string PARKING="PARKING"
    string ARRIVE_AT_STOP_LINE="ARRIVE_AT_STOP_LINE"
    string LANE_RECOVERY="LANE_RECOVERY"
    string INTERSECTION_RECOVERY="INTERSECTION_RECOVERY"
    string CALIBRATING="CALIBRATING"
    string CALIBRATING_CALC="CALIBRATING_CALC"

LEDPattern
^^^^^^^^^^

**File:** ``duckietown_msgs/LEDPattern.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    string[]  color_list
    std_msgs/ColorRGBA[]  rgb_vals
    int8[]    color_mask
    float32   frequency
    int8[]    frequency_mask


Twist2DStamped
^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/Twist2DStamped.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    float32 v
    float32 omega

WheelsCmd
^^^^^^^^^

**File:** ``duckietown_msgs/WheelsCmd.msg``

**Raw Message Definition:**

.. code-block:: text

    float32 vel_left
    float32 vel_right

WheelsCmdDBV2Stamped
^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/WheelsCmdDBV2Stamped.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    float32 gamma           #"vel_left" changed to "gamma", RFMH_2019_02_26
    float32 vel_wheel       #"vel_right" changed to "vel_wheel", RFMH_2019_02_26
    float32 trim            #included "trim" to be accessible in the wheels_driver_node as well, RFMH_2019_04_01

WheelsCmdStamped
^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/WheelsCmdStamped.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    float32 vel_left
    float32 vel_right


Image Pipeline
--------------

AntiInstagramThresholds
^^^^^^^^^^^^^^^^^^^^^^s

**File:** ``duckietown_msgs/AntiInstagramThresholds.msg``

**Raw Message Definition:**

.. code-block:: text

    int16[3] low
    int16[3] high

AprilTagDetection
^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/AprilTagDetection.msg``

**Raw Message Definition:**

.. code-block:: text

    geometry_msgs/Transform transform
    int32 tag_id
    string tag_family
    int32 hamming
    float32 decision_margin
    float32[9] homography
    float32[2] center
    float32[8] corners
    float32 pose_error

AprilTagDetectionArray
^^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/AprilTagDetectionArray.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    AprilTagDetection[] detections

AprilTagsWithInfos
^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/AprilTagsWithInfos.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    duckietown_msgs/AprilTagDetection[] detections
    duckietown_msgs/TagInfo[] infos

DisplayFragment
^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/AprilTagsWithInfos.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    duckietown_msgs/AprilTagDetection[] detections
    duckietown_msgs/TagInfo[] infos

LineFollowerStamped
^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/LineFollowerStamped.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    duckietown_msgs/AprilTagDetection[] detections
    duckietown_msgs/TagInfo[] infos

ObstacleImageDetection
^^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/ObstacleImageDetection.msg``

**Raw Message Definition:**

.. code-block:: text

    duckietown_msgs/Rect bounding_box
    duckietown_msgs/ObstacleType type

ObstacleImageDetectionList
^^^^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/ObstacleImageDetectionList.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    duckietown_msgs/ObstacleImageDetection[] list
    float32 imwidth
    float32 imheight

ObstacleProjectedDetection
^^^^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/ObstacleProjectedDetection.msg``

**Raw Message Definition:**

.. code-block:: text

    geometry_msgs/Point location
    duckietown_msgs/ObstacleType type
    float32 distance

ObstacleProjectedDetectionList
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/ObstacleProjectedDetectionList.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    duckietown_msgs/ObstacleProjectedDetection[] list


ObstacleType
^^^^^^^^^^^^

**File:** ``duckietown_msgs/ObstacleType.msg``

**Raw Message Definition:**

.. code-block:: text

    uint8 DUCKIE=0
    uint8 CONE=1
    uint8 type

Pixel
^^^^^

**File:** ``duckietown_msgs/Pixel.msg``

**Raw Message Definition:**

.. code-block:: text

    int32 u
    int32 v

Rect
^^^^

**File:** ``duckietown_msgs/Rect.msg``

**Raw Message Definition:**

.. code-block:: text

    # all in pixel coordinate
    # (x, y, w, h) defines a rectangle
    int32 x
    int32 y
    int32 w
    int32 h

Rects
^^^^^

**File:** ``duckietown_msgs/Rects.msg``

**Raw Message Definition:**

.. code-block:: text

    duckietown_msgs/Rect[] rects

SceneSegments
^^^^^^^^^^^^^

**File:** ``duckietown_msgs/SceneSegments.msg``

**Raw Message Definition:**

.. code-block:: text

    sensor_msgs/Image segimage
    duckietown_msgs/Rect[] rects

Segment
^^^^^^^

**File:** ``duckietown_msgs/Segment.msg``

**Raw Message Definition:**

.. code-block:: text

    uint8 WHITE=0
    uint8 YELLOW=1
    uint8 RED=2
    uint8 color
    duckietown_msgs/Vector2D[2] pixels_normalized
    duckietown_msgs/Vector2D normal

    geometry_msgs/Point[2] points

SegmentList
^^^^^^^^^^^

**File:** ``duckietown_msgs/SegmentList.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    duckietown_msgs/Segment[] segments

StreetNameDetection
^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/StreetNameDetection.msg``

**Raw Message Definition:**

.. code-block:: text

    #Mirrors TagDetection.h in the apriltags pkg
    bool good
    int32 id
    float32[] p
    float32[] cxy
    float32 observedPerimeter
    float32[] homography
    float32 orientation
    float32[] hxy
    geometry_msgs/Transform transform
    string text

TagInfo
^^^^^^^

**File:** ``duckietown_msgs/TagInfo.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    int32 id

    #(StreetName, TrafficSign, Localization, Vehicle)
    uint8 tag_type

    uint8 S_NAME=0
    uint8 SIGN=1
    uint8 LIGHT=2
    uint8 LOCALIZE=3
    uint8 VEHICLE=4

    string street_name

    uint8 traffic_sign_type
    # (12 possible traffic sign types)

    uint8 STOP=5
    uint8 YIELD=6
    uint8 NO_RIGHT_TURN=7
    uint8 NO_LEFT_TURN=8
    uint8 ONEWAY_RIGHT=9
    uint8 ONEWAY_LEFT=10
    uint8 FOUR_WAY=11
    uint8 RIGHT_T_INTERSECT=12
    uint8 LEFT_T_INTERSECT=13
    uint8 T_INTERSECTION=14
    uint8 DO_NOT_ENTER=15
    uint8 PEDESTRIAN=16
    uint8 T_LIGHT_AHEAD=17
    uint8 DUCK_CROSSING=18
    uint8 PARKING=19

    string vehicle_name

    # Just added a single number for location. Probably want to use Vector2D.msg, but I get errors when I try to add it.
    float32 location


VehicleCorners
^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/VehicleCorners.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    geometry_msgs/Point32[] corners
    std_msgs/Bool detection
    int32 H
    int32 W

Navigation
----------

CoordinationClearance
^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/CoordinationClearance.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    int8 status

    int8 NA=-1
    int8 WAIT=0
    int8 GO=1

CoordinationSignal
^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/CoordinationSignal.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header

    string signal

    # these must match with LED_protocol.yaml
    string OFF=light_off
    #string ON = light_on
    string ON=traffic_light_go
    string SIGNAL_A=CAR_SIGNAL_A
    string SIGNAL_B=CAR_SIGNAL_B
    string SIGNAL_C=CAR_SIGNAL_C
    string SIGNAL_GREEN = CAR_SIGNAL_GREEN
    string SIGNAL_PRIORITY = CAR_SIGNAL_PRIORITY
    string SIGNAL_SACRIFICE_FOR_PRIORITY = CAR_SIGNAL_SACRIFICE_FOR_PRIORITY

    string TL_GO_ALL=tl_go_all
    string TL_STOP_ALL=tl_stop_all
    string TL_GO_N=tl_go_N
    string TL_GO_S=tl_go_S
    string TL_GO_W=tl_go_W
    string TL_GO_E=tl_go_E
    string TL_YIELD=tl_yield

EpisodeStart
^^^^^^^^^^^^

**File:** ``duckietown_msgs/EpisodeStart.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    string episode_name
    string other_payload_yaml

IntersectionPose
^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/IntersectionPose.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    float32 x
    float32 y
    float32 theta
    uint8 type
    float32 likelihood

IntersectionPoseImg
^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/IntersectionPoseImg.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    float32 x
    float32 y
    float32 theta
    uint8 type
    float32 likelihood
    sensor_msgs/CompressedImage img

IntersectionPoseImgDebug
^^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/IntersectionPoseImgDebug.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    float32 x
    float32 y
    float32 theta
    uint8 type
    float32 likelihood
    float32 x_init
    float32 y_init
    float32 theta_init
    sensor_msgs/CompressedImage img

KinematicsParameters
^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/KinematicsParameters.msg``

**Raw Message Definition:**

.. code-block:: text

    float64[] parameters

KinematicsWeights
^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/KinematicsWeights.msg``

**Raw Message Definition:**

.. code-block:: text

    float64[] weights

LanePose
^^^^^^^^

**File:** ``duckietown_msgs/LanePose.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    float32 d   #lateral offset
    float32 d_ref #lateral offset reference
    float32 phi #heading error
    float32 phi_ref #heading error reference
    float32[4] d_phi_covariance
    float32 curvature
    float32 curvature_ref # Refernece Curvature
    float32 v_ref # Referenece Velocity
    int32 status #Status of duckietbot 0 if normal, 1 if error is encountered
    bool in_lane #Status of duckietbot in lane

    #Enum for status
    int32 NORMAL=0
    int32 ERROR=1

LEDInterpreter
^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/LEDInterpreter.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    bool vehicle
    bool trafficlight

MaintenanceState
^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/MaintenanceState.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    string state

    # pseudo constants
    string WAY_TO_MAINTENANCE="WAY_TO_MAINTENANCE"
    string WAY_TO_CHARGING="WAY_TO_CHARGING"
    string CHARGING="CHARGING"
    string WAY_TO_CALIBRATING="WAY_TO_CALIBRATING"
    string CALIBRATING="CALIBRATING"
    string WAY_TO_CITY="WAY_TO_CITY"
    string NONE="NONE"

Pose2DStamped
^^^^^^^^^^^^^

**File:** ``duckietown_msgs/Pose2DStamped.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    float64 x
    float64 y
    float64 theta

SignalsDetection
^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/SignalsDetection.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header

    # this is what we can see at the intersection:
    string front
    string right
    string left

    # For the first backoff approach
    # string led_detected
    # string no_led_detected

    # Each of these can be:
    string NO_CAR='no_car_detected'
    string SIGNAL_A='car_signal_A'
    string SIGNAL_B='car_signal_B'
    string SIGNAL_C='car_signal_C'
    string SIGNAL_PRIORITY='car_signal_priority'
    string SIGNAL_SACRIFICE_FOR_PRIORITY='car_signal_sacrifice_for_priority'

    string NO_CARS='no_cars_detected'
    string CARS   ='cars_detected'


    # Plus we can see the traffic light

    # for the moment we assume that no traffic light exists

    string traffic_light_state

    string NO_TRAFFIC_LIGHT='no_traffic_light'
    string STOP='tl_stop'
    string GO='tl_go'
    string YIELD='tl_yield'

StopLineReading
^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/StopLineReading.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    bool stop_line_detected
    bool at_stop_line
    geometry_msgs/Point stop_line_point #this is in the "lane frame"

StreetNames
^^^^^^^^^^^

**File:** ``duckietown_msgs/StreetNames.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    duckietown_msgs/StreetNameDetection[] detections

ThetaDotSample
^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/ThetaDotSample.msg``

**Raw Message Definition:**

.. code-block:: text

    float32 d_L
    float32 d_R
    float32 dt
    float32 theta_angle_pose_delta

Trajectory
^^^^^^^^^^

**File:** ``duckietown_msgs/Trajectory.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    geometry_msgs/Vector3Stamped[] pos
    geometry_msgs/Vector3Stamped[] vel
    geometry_msgs/Vector3Stamped[] acc
    geometry_msgs/Vector3Stamped[] jerk

TurnIDandType
^^^^^^^^^^^^^

**File:** ``duckietown_msgs/TurnIDandType.msg``

**Raw Message Definition:**

.. code-block:: text

    int16 tag_id
    int16 turn_type

VehiclePose
^^^^^^^^^^^

**File:** ``duckietown_msgs/VehiclePose.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    std_msgs/Float32 rho
    std_msgs/Float32 theta
    std_msgs/Float32 psi
    std_msgs/Bool detection

Vsample
^^^^^^^

**File:** ``duckietown_msgs/Vsample.msg``

**Raw Message Definition:**

.. code-block:: text

    float32 d_L
    float32 d_R
    float32 dt
    float32 theta_angle_pose_delta
    float32 x_axis_pose_delta
    float32 y_axis_pose_delta

Primitive
---------

BoolStamped
^^^^^^^^^^^

**File:** ``duckietown_msgs/BoolStamped.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    bool data

Vector2D
^^^^^^^^

**File:** ``duckietown_msgs/Vector2D.msg``

**Raw Message Definition:**

.. code-block:: text

    float32 x
    float32 y


ROS
---

NodeParameter
^^^^^^^^^^^^^

**File:** ``duckietown_msgs/NodeParameter.msg``

**Raw Message Definition:**

.. code-block:: text

    # Parameter type (this has to match duckietown.TopicType)
    uint8 PARAM_TYPE_UNKNOWN = 0
    uint8 PARAM_TYPE_STRING = 1
    uint8 PARAM_TYPE_INT = 2
    uint8 PARAM_TYPE_FLOAT = 3
    uint8 PARAM_TYPE_BOOL = 4

    string node         # Name of the node
    string name         # Name of the parameter (fully resolved)
    string help         # Description of the parameter
    uint8 type          # Type of the parameter (see PARAM_TYPE_X above)
    float32 min_value   # Min value (for type INT, UINT, and FLOAT)
    float32 max_value   # Max value (for type INT, UINT, and FLOAT)
    bool editable       # Editable (it means that the node will be notified for changes)

ParamTuner
^^^^^^^^^^

**File:** ``duckietown_msgs/ParamTuner.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    float32 cross_track_err
    float32 cross_track_integral
    float32 diff_cross_track_err
    float32 heading_err
    float32 heading_integral
    float32 diff_heading_err
    float32 dt

SourceTargetNodes
^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/SourceTargetNodes.msg``

**Raw Message Definition:**

.. code-block:: text

    string source_node
    string target_node


Sensors
-------

ButtonEvent
^^^^^^^^^^^

**File:** ``duckietown_msgs/ButtonEvent.msg``

**Raw Message Definition:**

.. code-block:: text

    uint8 EVENT_SINGLE_CLICK = 0
    uint8 EVENT_HELD_3SEC = 10
    uint8 EVENT_HELD_10SEC = 20

    uint8 event

DuckieSensor
^^^^^^^^^^^^

**File:** ``duckietown_msgs/DuckieSensor.msg``

**Raw Message Definition:**

.. code-block:: text

    # Sensors send value and type messages
    # For analog sensors value = 0..4095 and fvalue = 0.0..1.0
    # For digital sensors value= 0..1 and fvalue = 0.0
    uint16 value
    float32 fvalue
    bool is_analog
    string name

EncoderStamped
^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/EncoderStamped.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    float32 vel_encoder
    int64 count

LEDDetection
^^^^^^^^^^^^

**File:** ``duckietown_msgs/LEDDetection.msg``

**Raw Message Definition:**

.. code-block:: text

    time timestamp1		# initial timestamp of the camera stream used
    time timestamp2		# final timestamp of the camera stream used
    Vector2D pixels_normalized
    float32 frequency
    string color        # will be r, g or b
    float32 confidence  # some value of confidence for the detection (TBD)

    # for debug/visualization
    float64[] signal_ts
    float32[] signal
    float32[] fft_fs
    float32[] fft

LEDDetectionArray
^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/LEDDetectionArray.msg``

**Raw Message Definition:**

.. code-block:: text

    LEDDetection[] detections

LEDDetectionDebugInfo
^^^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/LEDDetectionDebugInfo.msg``

**Raw Message Definition:**

.. code-block:: text

    uint8 state # 0: idle, 1: capturing, 2: processing
    float32 capture_progress

    uint32[2] cell_size
    float32[4] crop_rect_norm

    sensor_msgs/CompressedImage variance_map
    Vector2D[] candidates

    LEDDetectionArray led_all_unfiltered


LightSensor
^^^^^^^^^^^

**File:** ``duckietown_msgs/LightSensor.msg``

**Raw Message Definition:**

.. code-block:: text

    Header header
    int32 r
    int32 g
    int32 b
    int32 c
    int32 real_lux
    int32 lux
    int32 temp

WheelEncoderStamped
^^^^^^^^^^^^^^^^^^^

**File:** ``duckietown_msgs/WheelEncoderStamped.msg``

**Raw Message Definition:**

.. code-block:: text

    # Enum: encoder type
    uint8 ENCODER_TYPE_ABSOLUTE = 0
    uint8 ENCODER_TYPE_INCREMENTAL = 1

    Header header
    int32 data
    uint16 resolution
    uint8 type
