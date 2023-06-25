ROS Package: ros_http_api
=========================

.. contents::

The ``ros_http_api`` package hosts the ROS node ``ros_http_api_node`` that exposes the ROS
environment as an HTTP API.


ROS Nodes
---------

ros_http_api_node
^^^^^^^^^^^^^^^^^

The ROS node ``ros_http_api_node`` exposes the ROS environment as an HTTP API.
The ROS HTTP API runs by default on any Duckietown device and can be reached at the
following URL,

    ``http://ROBOT_NAME.local/ros/ENDPOINT``

where ``ROBOT_NAME`` needs to be replaced with the name of your Robot while ``ENDPOINT``
is one of the endpoints exposed by the ROS HTTP API node (listed below).

Most of the endpoints were created with the idea of reproducing the same
CLI tools made available by ROS. For example, the CLI tool for getting the
list of ROS topics is ``rostopic list`` while its corresponding endpoint URL
is ``ros/topic/list``.


Responses
+++++++++

The ROS HTTP API returns data in JSON format.
Every response has the same structure. We distinguish between
**success** responses and **error** response.

A **success** response will look like the following:

.. code-block:: json

    {
        "status": "ok",
        "message": null,
        "data": "DATA"
    }

where the value of ``data`` is endpoint-dependent.

Conversely, an **error** response will look like the following:

.. code-block:: json

    {
        "status": "error",
        "message": "ERROR",
        "data": null
    }

where ``message`` is a string containing the error message.


Endpoints
+++++++++

The following endpoints are made available:


Endpoint: ``topic/list``
~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes no arguments and returns the list of all ROS topics
currently registered against the ROS Master node.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "topics": [
            "/topic1",
            "/topic2"
        ]
    }

where ``topics`` is the list of all ROS topics registered on the ROS
Master node.


Endpoint: ``topic/type``
~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``topic`` and returns the type of ROS
messages exchange over it.

The prototype of the endpoint URL is ``topic/type/<topic>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "topic": "/topic1",
        "message_type": "std_msgs/Int32"
    }

where,

- ``topic`` is the topic given as argument;
- ``message_type`` is the type of ROS messages published over the
  given topic;


Endpoint: ``topic/find``
~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``message_type`` and returns the list
of ROS topics that accept ROS messages of type ``message_type``.
The prototype of the endpoint URL is ``topic/find/<message_type>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "message_type": "std_msgs/Int32",
        "topics": [
            "/topic1",
            "/topic2"
        ]
    }

where,

- ``message_type`` is the given argument;
- ``topics`` is a list of ROS topics using the given message type;


Endpoint: ``topic/info``
~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``topic`` and returns information
about the given topic.

The prototype of the endpoint URL is ``topic/info/<topic>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "topic": "/topic1",
        "type": "GENERIC",
        "message_type": "std_msgs/Int32",
        "frequency": 30.0,
        "bandwidth": 119.2,
        "effective_frequency": 29.8,
        "publishers": [
            "/node1",
            "/node3"
        ],
        "subscribers": [
            "/node2",
            "/node4"
        ]
    }

where:

- ``topic`` is the topic given as argument;
- ``type`` is a topic type as defined in ``TopicType``
  in :py:mod:duckietown.dtros;
- ``message_type`` is a the type of ROS messages published over
  the given topic;
- ``frequency`` is the average frequency at which messages intended
  for this topic are instantiated.
  Topics that are not monitored using ``DTROS`` will have this field
  set to ``null``;
- ``bandwidth`` is the average bandwidth (in byte/s) needed for all
  the messages published over this topic (in one second) to flow from
  one publisher to one subscriber;
- ``effective_frequency`` is the average frequency at which messages
  instantiated for this topic are actually transferred (i.e.,
  messages that did not get dropped due to overflown queues).
  Also, topics with no subscribers will show an ``effective_frequency``
  of ``0`` as no messages are transferred while the ``frequency`` field
  will show the frequency at which messages are generated.
  Topics that are not monitored using ``DTROS`` will have this field
  set to ``null``;
- ``publishers`` and ``subscribers`` are list of ROS nodes
  publishing and subscribing to the given topic;

**TODO:** fix link to ``TopicType`` above


Endpoint: ``topic/hz``
~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``topic`` and returns the
average frequency at which messages are published over it.

The prototype of the endpoint URL is ``topic/hz/<topic>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "topic": "/topic1",
        "frequency": 29.9,
        "effective_frequency": 28.1,
        "secs_since_update": 1.2
    }

where:

- ``topic`` is the topic given as argument;
- ``frequency`` is the topic frequency in ``Hz``;
- ``effective_frequency`` is the average frequency at which messages
  instantiated for this topic are actually transferred
  (check the description in ``topic/info`` above);
- ``secs_since_update`` is the number of seconds elapsed since the
  given frequency value was computed;

.. note:: The frequency returned by this endpoint is computed
          differently than the one returned by the ROS cli
          ``rostopic hz``. While ``rostopic hz`` measures the
          frequency at which the machine performing the test
          can **receive** messages, this endpoint returns the frequency at which
          messages are actually **published**. These two numbers
          do not always coincide (e.g., when the network connecting
          origin to destination is slow).

**TODO:** fix link to ``topic/info`` above


Endpoint: ``topic/bw``
~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``topic`` and returns the
bandwidth needed to allow all the messages published on the it
to flow.

The prototype of the endpoint URL is ``topic/bw/<topic>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "topic": "/topic1",
        "bandwidth": 240.0,
        "secs_since_update": 0.2
    }

where:

- ``topic`` is the topic given as argument;
- ``frequency`` is the topic bandwidth in ``bytes/sec``;
- ``secs_since_update`` is the number of seconds elapsed since the
  given bandwidth value was measured;

.. note:: The bandwidth returned by this endpoint is computed
          differently than the one returned by the ROS cli
          ``rostopic bw``. While ``rostopic bw`` measures the
          total size of messages **received** in one second by the
          machine performing the measurement, this endpoint returns
          the bandwidth needed to accomodate all the messages
          **published** by all the publishers of this topic.
          These two numbers do not always coincide (e.g., when the
          network connecting origin to destination is slow).


Endpoint: ``topic/publishers``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``topic`` and returns the list
of ROS nodes publishing on the given topic.

The prototype of the endpoint URL is ``topic/publishers/<topic>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "topic": "/topic1",
        "publishers": [
            "/node1",
            "/node3"
        ]
    }

where:

- ``topic`` is the topic given as argument;
- ``publishers`` is the list of ROS nodes publishing on the given topic;


Endpoint: ``topic/subscribers``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``topic`` and returns the list
of ROS nodes subscribing to the given topic.

The prototype of the endpoint URL is ``topic/subscribers/<topic>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "topic": "/topic1",
        "subscribers": [
            "/node2",
            "/node4"
        ]
    }

where:

- ``topic`` is the topic given as argument;
- ``subscribers`` is the list of ROS nodes subscribing to the given topic;


Endpoint: ``topic/dttype``
~~~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``topic`` and returns its type
as defined in ``TopicType`` in :py:mod:duckietown.dtros;

The prototype of the endpoint URL is ``topic/dttype/<topic>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "topic": "/topic1",
        "type": "DIAGNOSTICS"
    }

where:

- ``topic`` is the topic given as argument;
- ``type`` is a topic type as defined in ``TopicType``
  in :py:mod:duckietown.dtros;

**TODO:** fix link(s) to ``TopicType`` above


Endpoint: ``node/list``
~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes no arguments and returns the list of ROS nodes
currently registered with the ROS Master node.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "nodes": [
            "/node1",
            "/node2"
        ]
    }

where:

- ``nodes`` is the list of ROS nodes currently registered with the
  ROS Master node.


Endpoint: ``node/info``
~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``node`` and returns information
about the given node.

The prototype of the endpoint URL is ``node/info/<node>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "node": "/node1",
        "type": "MAPPING",
        "enabled": true,
        "health": "STARTED",
        "health_value": 6,
        "health_reason": "None",
        "machine": "duckiebot01",
        "module_instance": "33362ec7a9a5e7f9d5421b2642e44040bf78e719e90e0efd3601af28ce654ccb",
        "module_type": "dt-core",
        "topics": [
            "/topic1",
            "/topic2"
        ],
        "services": [
            "/service1",
            "/service2"
        ],
        "parameters": [
            "/param1",
            "/param2",
            "/param3"
        ]


where:

- ``node`` is the node given as argument;
- ``type`` is a node type as defined in ``NodeType``
  in :py:mod:duckietown.dtros;
- ``enabled`` indicates whether the node is currently active
  (nodes in Duckietown can be switched off);
- ``health`` reports the health of the node, allowed values are those
  defined in ``NodeHealth`` in :py:mod:duckietown.dtros;
- ``health_value`` is the numeric value indicating the health of the node;
  allowed values are those defined in ``NodeHealth`` in :py:mod:duckietown.dtros;
- ``health_reason`` is a string containing an explanation when the
  node is in an unhealthy state;
- ``machine`` is the hostname of the computer this node is running on;
- ``module_instance`` is the ID of the module (Docker container) hosting
  this node;
- ``module_type`` is the name of the Docker image the container was
  instantiated from (e.g., dt-core);
- ``topics`` is the list of topics this node publishes or
  subscribes to;
- ``services`` is the list of services provided by the node;
- ``parameters`` is the list of parameters used by the node;

**TODO:** fix link to ``NodeType`` above


Endpoint: ``node/topics``
~~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``node`` and returns the list
of topics it subscribes or publishes to.

The prototype of the endpoint URL is ``node/topics/<node>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "node": "/node1",
        "topics": {
            "/topic1": {
                "direction": "INBOUND"
            },
            "/topic2": {
                "direction": "OUTBOUND"
            }
        }
    }

where:

- ``node`` is the node given as argument;
- ``topics`` is the list of topics this node publishes or
  subscribes to;
- ``topics.<topic>.direction`` is one of ``INBOUND``, ``OUTBOUND`` indicating
  whether the node subscribes to the topic (``INBOUND``) or publishes it
  (``OUTBOUND``).


Endpoint: ``node/params``
~~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``node`` and returns the list
of ROS parameters used by the node.

The prototype of the endpoint URL is ``node/params/<node>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "node": "/node1",
        "parameters": [
            "/param1",
            "/param2",
            "/param3"
        ]
    }

where:

- ``node`` is the node given as argument;
- ``parameters`` is the list of parameters used by the node;


Endpoint: ``node/services``
~~~~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``node`` and returns the
list of services provided by the node.

The prototype of the endpoint URL is ``node/services/<node>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "node": "/node1",
        "services": [
            "/service1",
            "/service2"
        ]
    }

where:

- ``node`` is the node given as argument;
- ``services`` is the list of services provided by the node;


Endpoint: ``param/list``
~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes one optional arguments ``namespace`` and
returns the list of ROS parameters currently registered with
the ROS Parameter Server node. If the ``namespace`` is provided
only those parameters that have a prefix matching the namespace
are returned. This is useful when we want to fetch the list of
parameters grouped together under the same namespace, e.g.,
parameters belonging to the same module or node.

The prototype of the endpoint URL is ``param/list/[<namespace>]``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "parameters": [
            "/param1",
            "/param2"
        ]
    }

where:

- ``parameters`` is the list of ROS parameters currently
  registered with the ROS Master node (possibly filter by
  a given namespace).


Endpoint: ``param/get``
~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** arguments ``parameter`` and
returns its current value, fetched from the ROS Parameter
Server node.

The prototype of the endpoint URL is ``param/get/<parameter>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "parameter": "/param1",
        "value": "value1"
    }

where:

- ``parameter`` is the ROS parameter name given as argument;
- ``value`` is the parameter's value. Its type is parameter-dependent;


Endpoint: ``param/info``
~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** arguments ``parameter`` and
returns information about the corresponding ROS parameter.

The prototype of the endpoint URL is ``param/info/<parameter>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "param": "/param1",
        "value": "value1"
        "editable": false,
        "max_value": 30,
        "min_value": 20,
        "type": "MAPPING",
    }

where:

- ``parameter`` is the ROS parameter name given as argument;
- ``value`` is the parameter's value. Its type is parameter-dependent;
- ``editable`` indicates whether the parameter supports runtime updates;
- ``max_value`` indicates the maximum value supported by the parameter (-1 if not set);
- ``min_value`` indicates the minimum value supported by the parameter (-1 if not set);
- ``type`` is a parameter type as defined in ``ParamType``
  in :py:mod:duckietown.dtros;


Endpoint: ``service/list``
~~~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes no arguments and returns the list of all ROS
services currently registered with the ROS Master node.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "services": [
            "/service1",
            "/service2"
        ]
    }

where ``services`` is the list of all ROS services registered
with the ROS Master node.


Endpoint: ``service/type``
~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``service`` and returns the
type of ROS messages exchange over it.

The prototype of the endpoint URL is ``service/type/<service>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "service": "/service1",
        "message_type": "std_srvs/SetBoolRequest"
    }

where,

- ``service`` is the service given as argument;
- ``message_type`` is the type of ROS message used to request
  the service execution;


Endpoint: ``service/find``
~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``message_type`` and returns
the list of ROS services that accept ROS messages of type
``message_type``.
The prototype of the endpoint URL is ``service/find/<message_type>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "message_type": "std_srvs/SetBoolRequest",
        "services": [
            "/service1",
            "/service2"
        ]
    }

where,

- ``message_type`` is the given argument;
- ``services`` is a list of ROS services using the given message
  type;


Endpoint: ``service/info``
~~~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``service`` and returns
information about the corresponding ROS service.

The prototype of the endpoint URL is ``service/info/<service>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "service": "/service1",
        "message_type": "std_msgs/Int32",
        "providers": [
            "/node1",
            "/node3"
        ]
    }

where:

- ``service`` is the service given as argument;
- ``message_type`` is a the type of ROS messages used by the
  given service;
- ``providers`` is the list of ROS nodes providing the
  given service;


Endpoint: ``service/providers``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This endpoint takes **one** argument ``service`` and returns the
list of ROS nodes providing the given service.

The prototype of the endpoint URL is ``service/providers/<service>``.

An example of the content of the ``data`` field is

.. code-block:: json

    {
        "service": "/service1",
        "providers": [
            "/node1",
            "/node3"
        ]
    }

where:

- ``service`` is the service given as argument;
- ``providers`` is the list of ROS nodes providing the given service;


Endpoint: ``graph``
~~~~~~~~~~~~~~~~~~~

This endpoint combines most of the data returned by the single
endpoints defined above and returns it in one call.
It takes no arguments.

Inspired by the ``rosgraph`` cli tool made available by ROS, this
endpoint returns information about nodes, topics, which nodes
publish which topics and which nodes subscribe to which topics.

This endpoint is designed for applications that need to obtain
a complete picture of the entire ROS network at once. The output
of this endpoint is structured in such a way that makes it easy
to render the ROS network as a graph (similarly to ``rqt_graph``).

An example of the content of the ``data`` field for a ROS network
comprising of two nodes and two topics is

.. code-block:: json

    {
        "graph": {
            "nodes": [
                "/node1",
                "/node2"
            ],
            "edges": {
                "node_to_topic": [
                    {
                        "from": "/node1",
                        "to": "/topicA"
                    },
                    {
                        "from": "/node2",
                        "to": "/topicB"
                    }
                ],
                "topic_to_node": [
                    {
                        "from": "/topicA",
                        "to": "/node2"
                    }
                ],
                "node_to_node": [
                    {
                        "from": "/node1",
                        "middle": "/topicA",
                        "to": "/node2"
                    }
                ],
                "topic_to_topic": [
                    {
                        "from": "/topicA",
                        "middle": "/node2",
                        "to": "/topicB"
                    }
                ]
            }
        },
        "nodes": {
            "/node1": {
                "enabled": true,
                "health": "STARTED",
                "health_reason": "None",
                "health_stamp": 1593541120.0,
                "health_value": 6,
                "machine": "autobot04",
                "module_instance": "3fa4def01fcaa5535f8fab9e1625aa4c7b91ee8362cddba6de066bb576856c26",
                "module_type": "dt-duckiebot-interface",
                "type": "DRIVER"
            },
            "/node2": {
                "...": "..."
            }
        },
        "topics": {
            "/topicA": {
                "bandwidth": 0.0,
                "effective_frequency": 0.0,
                "frequency": 28.74346351623535,
                "message_type": null,
                "type": "DRIVER"
            },
            "/topicB": {
                "...": "..."
            }
        }
    }

where:

- ``graph`` is an object representing a ROS graph;
- ``graph.nodes`` is the list of all ROS nodes;
- ``graph.edges`` is an object containing four types of possible edges:
  ``node_to_topic``, ``topic_to_node``, ``node_to_node``, ``topic_to_topic``;
- ``graph.edges.node_to_topic`` is the list of node to topic connections.
  This list contains one entry for each active rospy.Publisher
  object in the network;
- ``graph.edges.topic_to_node`` is the list of topic to node connections.
  This list contains one entry for each active rospy.Subscriber
  object in the network;
- ``graph.edges.node_to_node`` is the list of node to node connections.
  This list contains one entry for each pair of nodes that talk
  directly through a ROS topic.
- ``graph.edges.topic_to_topic`` is the list of topic to topic connections.
  This list contains one entry for each tuple
  (``/topicX``, ``/nodeY``, ``/topicZ``) such that ``/nodeY`` has
  both a listener on ``/topicX`` and a publisher on ``/topicZ``.
  This list wants to capture the idea of topics that **might**
  influence one another.
- ``nodes`` is a dictionary of all ROS nodes; Check the endpoint ``node/info``
  for further information about the fields contained in this object.
- ``topics`` is a dictionary of all ROS topics; Check the endpoint ``topic/info``
  for further information about the fields contained in this object.
