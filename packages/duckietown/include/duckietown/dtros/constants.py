from enum import Enum

DIAGNOSTICS_ENABLED = True

DIAGNOSTICS_ROS_NODE_PUB_EVERY_SEC = 30.0
DIAGNOSTICS_ROS_NODE_TOPIC = '/diagnostics/ros/node'

DIAGNOSTICS_ROS_TOPICS_PUB_EVERY_SEC = 10.0
DIAGNOSTICS_ROS_TOPICS_TOPIC = '/diagnostics/ros/topics'

DIAGNOSTICS_ROS_PARAMETERS_PUB_EVERY_SEC = 30.0
DIAGNOSTICS_ROS_PARAMETERS_TOPIC = '/diagnostics/ros/parameters'

DIAGNOSTICS_ROS_LINKS_PUB_EVERY_SEC = 10.0
DIAGNOSTICS_ROS_LINKS_TOPIC = '/diagnostics/ros/links'

NODE_SWITCH_SERVICE_NAME = 'switch'
NODE_GET_PARAM_SERVICE_NAME = 'get_parameters_list'
NODE_REQUEST_PARAM_UPDATE_SERVICE_NAME = 'request_parameters_update'


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
    DIAGNOSTICS = 20


TopicType = ModuleType


# NOTE: this has to match duckietown_msgs.msg.NodeParameter
class ParamType(Enum):
    UNKNOWN = 0
    STRING = 1
    INT = 2
    FLOAT = 3
    BOOL = 4
    LIST = 5
    DICT = 6

    _type_to_ptype = {
        UNKNOWN: lambda x: x,
        STRING: str,
        INT: int,
        FLOAT: float,
        BOOL: bool,
        LIST: lambda x: x,
        DICT: lambda x: x
    }

    _ptype_to_type = {
        str: STRING,
        int: INT,
        float: FLOAT,
        bool: BOOL,
        list: LIST,
        tuple: LIST,
        dict: DICT
    }

    @classmethod
    def parse(cls, param_type, value):
        if value is None:
            return None
        if not isinstance(param_type, ParamType):
            raise ValueError("Argument 'param_type' must be of type ParamType. "
                             "Got %s instead." % str(type(param_type)))
        # ---
        return cls._type_to_ptype.value[param_type.value](value)

    @classmethod
    def guess_type(cls, param_value):
        if type(param_value) in cls._ptype_to_type.value:
            return cls(cls._ptype_to_type.value[type(param_value)])
        return cls.UNKNOWN


class NodeHealth(Enum):
    UNKNOWN = 0
    STARTING = 1
    HEALTHY = 2
    WARNING = 3
    ERROR = 4
    FATAL = 5
