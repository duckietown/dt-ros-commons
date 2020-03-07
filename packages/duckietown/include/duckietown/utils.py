import gc
import rospy

__rh = None


def get_ros_handler(force=False):
    """
    TODO: This is a temporary function, I'm not sure we will need it.
    :param force:
    :return:
    """
    global __rh
    if __rh is not None and not force:
        return __rh
    rhs = [
        e for e in gc.get_objects()
        if isinstance(e, rospy.impl.masterslave.ROSHandler)
    ]
    if len(rhs) == 0:
        return None
    if len(rhs) == 1:
        __rh = rhs[0]
    if len(rhs) > 1:
        print('WARNING: Two or more ROS Handlers were found in memory. If you happen to see this, '
              'please, post a message on https://github.com/duckietown/dt-ros-commons/issues/24.')
    return __rh


def get_namespace(level):
    node_name = rospy.get_name()
    namespace_comps = node_name.lstrip('/').split('/')
    if level > len(namespace_comps):
        level = len(namespace_comps)
    return '/{:s}'.format('/'.join(namespace_comps[:level]))
