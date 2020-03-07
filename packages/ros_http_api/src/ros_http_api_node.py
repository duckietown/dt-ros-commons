#!/usr/bin/env python

import sys
import signal

from dt_ros_api import ROS_HTTP_API

from threading import Thread


def paramUpdate(*args, **kwargs):
    print('UPDATE')


def signal_handler(sig, frame):
    sys.exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    api = ROS_HTTP_API()

    # t = Thread(target=api.run, kwargs={'debug': True, 'host': '0.0.0.0'})
    # t.start()
    # t.join()

    api.run(debug=True, host='0.0.0.0')

    # import gc
    # import rospy
    # import networkx as nx
    #
    # G = nx.Graph()
    # iapi = id(api)
    # path_found = False
    #
    # rh = [e for e in gc.get_objects() if type(e) == rospy.impl.masterslave.ROSHandler][0]
    # irh = id(rh)
    #
    # from types import ModuleType
    # id_to_object_type = {
    #     id(e): (type(e) if not isinstance(e, ModuleType) else e) for e in gc.get_objects()
    # }
    #
    #
    #
    # def _extend_graph(c, l):
    #     ic = id(c)
    #     if l > 100:
    #         return False
    #
    #     # if isinstance(c, ModuleType):
    #     #     print(c)
    #
    #
    #     if ic == iapi:
    #         print('Path found!')
    #         print(
    #             [
    #                 id_to_object_type[e] for e in
    #                 list(nx.algorithms.simple_paths.all_simple_paths(G, iapi, irh))[0]
    #             ]
    #         )
    #         return False
    #     for p in gc.get_referrers(c):
    #         ip = id(p)
    #         if ip == iapi or not G.has_node(ip):
    #             G.add_node(ip)
    #             G.add_edge(ip, ic)
    #             if _extend_graph(p, l+1):
    #                 return
    #
    # G.add_node(irh)
    # _extend_graph(rh, 0)
    #
    # print(G.number_of_nodes())



