import rospy
from duckietown_msgs.msg import DuckiematrixConnectorsDescription
from flask import Blueprint, request

from dt_ros_api.utils import response_ok, response_error

duckiematrix = Blueprint('duckiematrix', __name__)


_pub = rospy.Publisher(
    "~duckiematrix/connect",
    DuckiematrixConnectorsDescription,
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
    name = args.get("name", "local")
    data_in = args.get("data_in", None)
    data_out = args.get("data_out", None)
    # check parameters
    if data_in is None:
        return response_error("Parameter 'data_in' is required.")
    if data_out is None:
        return response_error("Parameter 'data_out' is required.")
    # make a ROS message with the info
    msg = DuckiematrixConnectorsDescription(
        name=name,
        data_in_uri=data_in,
        data_out_uri=data_out,
    )
    # publish message
    _pub.publish(msg)
    # return current API duckiematrix
    return response_ok({
        'name': name,
        'data_in': data_in,
        'data_out': data_out,
    })
