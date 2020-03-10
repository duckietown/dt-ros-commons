#!/usr/bin/env python

import rospy

from parameter_server_helper import ParameterServerHelper


if __name__ == '__main__':
    param_server_helper = ParameterServerHelper()
    rospy.spin()


