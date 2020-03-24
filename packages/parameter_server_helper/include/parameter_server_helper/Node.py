import os
import rospy
from threading import Semaphore, Thread

from duckietown_msgs.srv import NodeGetParamsList
from duckietown.dtros.constants import \
    NODE_GET_PARAM_SERVICE_NAME,\
    NODE_REQUEST_PARAM_UPDATE_SERVICE_NAME

from .constants import \
    FETCH_NEW_PARAMS_EVERY_SECS,\
    NUM_SUCCESSFUL_PARAM_FETCHING_ACTIONS,\
    NUM_TRIALS_REQUEST_PARAM_UPDATE_ACTION


class Node:

    def __init__(self, name):
        self.name = name
        self._parameters = set()
        self._parameters_lock = Semaphore(1)
        self._parameters_num_fetch_ops = 0
        # start updating the list of parameters
        self._update_nodes_list_timer = rospy.Timer(
            period=rospy.Duration.from_sec(FETCH_NEW_PARAMS_EVERY_SECS),
            callback=self.update_parameters_list,
            oneshot=False
        )
        # spin right away
        self.update_parameters_list()

    def request_update(self, param_name):
        if param_name not in self._parameters:
            return
        # ---
        rospy.logdebug('Node "%s" being notified about parameter "%s"' % (self.name, param_name))
        t = Thread(target=self._request_update_param, args=(param_name,))
        t.start()

    def shutdown(self):
        self._update_nodes_list_timer.shutdown()

    def _request_update_param(self, param_name):
        ntrial = 0
        update_param_service_name = os.path.join(self.name, NODE_REQUEST_PARAM_UPDATE_SERVICE_NAME)
        while ntrial < NUM_TRIALS_REQUEST_PARAM_UPDATE_ACTION:
            # try to notify the node
            try:
                # wait for service
                rospy.wait_for_service(update_param_service_name, 5.0)
                update_param_service = rospy.ServiceProxy(
                    update_param_service_name, NodeGetParamsList
                )
                # call service
                update_param_service(parameter=param_name)
                # ---
                rospy.logdebug('Node "%s" notified about parameter "%s"' % (self.name, param_name))
            finally:
                ntrial += 1

    def update_parameters_list(self, *args, **kwargs):
        params = []
        if 'params' in kwargs:
            # this call comes from the diagnostics subscriber so it has the list of params already
            params = kwargs['params']
        else:
            # this was called by the Timer, we need to interrogate the node for the list of params
            rospy.logdebug('Node "%s", updating list of parameters (%d/%d)' % (
                self.name, self._parameters_num_fetch_ops+1, NUM_SUCCESSFUL_PARAM_FETCHING_ACTIONS
            ))
            if self._parameters_num_fetch_ops > NUM_SUCCESSFUL_PARAM_FETCHING_ACTIONS:
                # stop the timer
                self._update_nodes_list_timer.shutdown()
                return
            # try fetching the params list
            error_detected = False
            get_params_service_name = os.path.join(self.name, NODE_GET_PARAM_SERVICE_NAME)
            try:
                rospy.wait_for_service(get_params_service_name, 5.0)
            except rospy.exceptions.ROSException:
                return
            # call service
            try:
                get_params_service = rospy.ServiceProxy(get_params_service_name, NodeGetParamsList)
                res = get_params_service()
                params = [p.name for p in res.parameters]
            except Exception:
                error_detected = True
            # stop if an error occurred
            if error_detected:
                return
            # mark this attemp as successful
            self._parameters_num_fetch_ops += 1
        # merge new list of parameters with existing set
        self._parameters_lock.acquire()
        try:
            # find new parameters (need to add)
            new_params = set(params).difference(self._parameters)
            if len(new_params):
                rospy.logdebug('Node "%s", found %d new parameters:\n\t%s' % (
                    self.name, len(new_params), '\n\t'.join(new_params)
                ))
            # find old parameters (need to remove)
            old_params = set(self._parameters).difference(params)
            if len(new_params):
                rospy.logdebug('Node "%s", found %d old parameters:\n\t%s' % (
                    self.name, len(old_params), '\n\t'.join(old_params)
                ))
            # add new params
            for param in new_params:
                self._parameters.add(param)
                # TODO: this is where we subscribe to this new parameter
            # remove old params
            for param in old_params:
                self._parameters.remove(param)
        finally:
            self._parameters_lock.release()
