import rospy

from . import get_instance
from .constants import ParamType

MIN_MAX_SUPPORTED_TYPES = [ParamType.INT, ParamType.FLOAT]


class DTParam:

    def __init__(self, name, default=None, param_type=ParamType.UNKNOWN, min_value=None, max_value=None):
        self._name = rospy.names.resolve_name(name)
        if not isinstance(param_type, ParamType):
            raise ValueError(
                "Parameter 'param_type' must be an instance of duckietown.ParamType. "
                'Got %s instead.' % str(type(param_type))
            )
        self._type = param_type
        # parse optional args
        # - min value
        if min_value is not None and param_type not in MIN_MAX_SUPPORTED_TYPES:
            raise ValueError(
                "Parameter 'min_value' not supported for parameter of type '%s'." % param_type.name
            )
        self._min_value = ParamType.parse(param_type, min_value)
        # - max value
        if max_value is not None and param_type not in MIN_MAX_SUPPORTED_TYPES:
            raise ValueError(
                "Parameter 'max_value' not supported for parameter of type '%s'." % param_type.name
            )
        self._max_value = ParamType.parse(param_type, max_value)
        # - default value
        self._default_value = ParamType.parse(param_type, default)
        if self._default_value is not None:
            # verify lower-bound
            if self._min_value is not None and self._default_value < self._min_value:
                raise ValueError(
                    "Given default value %s is below the min_value %s for parameter '%s'" % (
                        str(self._default_value), str(self._min_value), name
                    )
                )
            # verify upper-bound
            if self._max_value is not None and self._default_value > self._max_value:
                raise ValueError(
                    "Given default value %s is above the max_value %s for parameter '%s'" % (
                        str(self._default_value), str(self._max_value), name
                    )
                )
        # ---
        node = get_instance()
        if node is None:
            raise ValueError(
                'You cannot create a DTParam object before initializing a DTROS object'
            )
        # get parameter value
        if rospy.has_param(self._name):
            self._value = rospy.get_param(self._name)
        else:
            self._value = self._default_value
            rospy.set_param(self._name, self._default_value)
        # add param to current node
        node._add_param(self)

    def force_update(self):
        # get parameter value
        self._value = rospy.get_param(self._name, self._default_value)

    def options(self):
        options = {}
        # min value
        if self.min_value is not None:
            options['min_value'] = self.min_value
        # max value
        if self.max_value is not None:
            options['max_value'] = self.max_value
        # ---
        return options

    @property
    def name(self):
        return self._name

    @property
    def value(self):
        return self._value

    @property
    def default(self):
        return self._default_value

    @property
    def min_value(self):
        return self._min_value

    @property
    def max_value(self):
        return self._max_value

    @property
    def type(self):
        return self._type
