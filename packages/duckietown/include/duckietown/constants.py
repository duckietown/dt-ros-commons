from enum import Enum

DIAGNOSTICS_ENABLED = True

DIAGNOSTICS_ROS_TOPICS_PUB_EVERY_SEC = 2.0
DIAGNOSTICS_ROS_TOPICS_TOPIC = '/diagnostics/ros/topics'

DIAGNOSTICS_ROS_PARAMETERS_PUB_EVERY_SEC = 2.0
DIAGNOSTICS_ROS_PARAMETERS_TOPIC = '/diagnostics/ros/parameters'

DIAGNOSTICS_ROS_LINKS_PUB_EVERY_SEC = 2.0
DIAGNOSTICS_ROS_LINKS_TOPIC = '/diagnostics/ros/links'


class TopicDirection(Enum):
    INBOUND = 0
    OUTBOUND = 1


# NOTE: this has to match duckietown_msgs.msg.DiagnosticsRosTopic
class ModuleType(Enum):
    GENERIC = 0
    DRIVER = 1
    PERCEPTION = 2
    CONTROL = 3
    PLANNING = 4
    LOCALIZATION = 5
    MAPPING = 6
    SWARM = 7
    BEHAVIOR = 8
    VISUALIZATION = 9


TopicType = ModuleType
