import rospy, rosgraph, rostopic as rt
from flask import Blueprint, jsonify, request, abort

import uuid

from dt_ros_api.utils import \
    response_ok,\
    response_error

rostopic = Blueprint('topic', __name__)

# API handlers
#
# - topic/list      (Supported)
# - topic/type      (Supported)
# - topic/find      (Supported)
# - topic/info      (Supported)
# - topic/bw        (Not Supported)     * Will be supported with diagnostics
# - topic/delay     (Not Supported)     * Will be supported with diagnostics
# - topic/hz        (Not Supported)     * Will be supported with diagnostics
# - topic/echo      (Not Supported)
# - topic/pub       (Not Supported)
#


@rostopic.route('/topic/list')
def _list():
    try:
        return response_ok({
            'topics': {
                _rostopic: _rostopic_type
                for _rostopic, _rostopic_type in rospy.get_published_topics()
            }
        })
    except Exception as e:
        return response_error(str(e))


@rostopic.route('/topic/type/<path:topic>')
def _type(topic):
    topic = '/' + topic
    try:
        _topic_type, _, _ = rt.get_topic_type(topic)
        if _topic_type:
            return response_ok({
                'type': _topic_type
            })
        else:
            return response_error("Topic '{:s}' not found".format(topic))
    except Exception as e:
        return response_error(str(e))


@rostopic.route('/topic/find/<path:msg_type>')
def _find(msg_type):
    try:
        return response_ok({
            'topics': rt.find_by_type(msg_type)
        })
    except Exception as e:
        return response_error(str(e))


@rostopic.route('/topic/info/<path:topic>')
def _info(topic):
    topic = '/' + topic
    try:
        master = rosgraph.Master(uuid.uuid4())
        pubs, subs, _ = master.getSystemState()
        info = {
            'topic': topic,
            'type': rt.get_topic_type(topic)[0]
        }
        # get publishers
        for _topic, _topic_pubs in pubs:
            if _topic == topic:
                info['publishers'] = _topic_pubs
                break
        # get subscribers
        for _topic, _topic_subs in subs:
            if _topic == topic:
                info['subscribers'] = _topic_subs
                break
        # return
        return response_ok(info)
    except Exception as e:
        return response_error(str(e))



