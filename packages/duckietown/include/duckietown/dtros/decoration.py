# This file performs the decoration of rospy upon import

# TODO: update this to point to the documentation about DTROS once ready
DOCS_DTROS = 'http://docs.duckietown.org'

import rospy

# import objects decorating rospy
from .dtpublisher import DTPublisher
from .dtsubscriber import DTSubscriber


def rospy_decorate():
    # decorate:
    # - rospy.init_node
    setattr(rospy, 'init_node', __rospy__init_node__)
    # - rospy.Publisher
    setattr(rospy, 'Publisher', DTPublisher)
    # - rospy.Subscriber
    setattr(rospy, 'Subscriber', DTSubscriber)


def __rospy__init_node__(*args, **kwargs):
    if '__dtros__' not in kwargs:
        print('[WARNING]: You are calling the function rospy.init_node() directly. '
              'This is fine, but your are missing out a lot of Duckietown features '
              'by not using the class DTROS instead. Check out the documentation '
              'at %s' % DOCS_DTROS)
    else:
        del kwargs['__dtros__']
    # ---
    return rospy.__init_node__(*args, **kwargs)