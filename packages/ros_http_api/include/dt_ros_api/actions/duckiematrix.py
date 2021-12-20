import rospy
from duckietown_msgs.msg import DuckiematrixLinkDescription
from flask import Blueprint, request

from dt_ros_api.utils import response_ok, response_error

duckiematrix = Blueprint('duckiematrix', __name__)


_pub = rospy.Publisher(
    "~duckiematrix/connect",
    DuckiematrixLinkDescription,
    queue_size=10,
    latch=True
)


@duckiematrix.route('/duckiematrix/connect', methods=['POST', 'GET'])
def _duckiematrix_start():
    # tell ROS nodes to connect to a duckiematrix engine
    if request.method == "POST":
        args = request.form
    else:
        args = request.args
    # get name, and URIs
    matrix = args.get("matrix", "local")
    uri = args.get("uri", None)
    entity = args.get("entity", None)
    # check parameters
    if uri is None:
        return response_error("Parameter 'uri' is required.")
    if entity is None:
        return response_error("Parameter 'entity' is required.")
    # make a ROS message with the info
    msg = DuckiematrixLinkDescription(
        matrix=matrix,
        uri=uri,
        entity=entity,
    )
    # publish message
    _pub.publish(msg)
    # return current API duckiematrix
    return response_ok({
        'matrix': matrix,
        'uri': uri,
        'entity': entity,
    })
