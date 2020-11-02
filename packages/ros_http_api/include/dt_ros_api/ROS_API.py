from flask import Flask
from flask_cors import CORS

from .actions.topic import rostopic
from .actions.node import rosnode
from .actions.param import rosparam
from .actions.service import rosservice
from .actions.graph import rosgraph


class ROS_HTTP_API(Flask):
    def __init__(self):
        super(ROS_HTTP_API, self).__init__(__name__)
        self.register_blueprint(rostopic)
        self.register_blueprint(rosnode)
        self.register_blueprint(rosparam)
        self.register_blueprint(rosservice)
        self.register_blueprint(rosgraph)
        # apply CORS settings
        CORS(self)
