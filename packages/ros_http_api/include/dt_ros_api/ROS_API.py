from flask import Flask

from .actions.topic import rostopic
from .diagnostics import ROSDiagnosticsListener


class ROS_HTTP_API(object):

    def __init__(self):
        # super(ROS_HTTP_API, self).__init__(__name__)
        # self.register_blueprint(rostopic)
        self.ros_listener = ROSDiagnosticsListener()
