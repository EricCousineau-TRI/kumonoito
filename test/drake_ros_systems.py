"""
Provides very simple pure Python ROS systems.

Note:
    These are slightly different than the neighboring C++ systems.
"""

from threading import Lock

import rospy

from pydrake.systems.framework import (
    AbstractValue,
    LeafSystem,
    PublishEvent,
    TriggerType,
)


class RosPublisherSystem(LeafSystem):
    """Publishes a message from an input port at a given interval."""
    def __init__(self, topic, msg_type, period_sec, queue_size=10):
        LeafSystem.__init__(self)
        self.topic = topic
        self.msg_type = msg_type
        self._pub = rospy.Publisher(topic, msg_type, queue_size=queue_size)
        self._input = self._DeclareAbstractInputPort(
            "msg", AbstractValue.Make(msg_type())).get_index()
        self._DeclarePeriodicEvent(
            period_sec=period_sec, offset_sec=0.,
            event=PublishEvent(TriggerType.kPeriodic, self._ros_publish))
        self._publish_message_count = 0

    def _ros_publish(self, context, event):
        msg = self.EvalAbstractInput(context, self._input).get_value()
        self._pub.publish(msg)
        self._publish_message_count += 1

    def num_messages(self):
        """Number of messages published."""
        return self._publish_message_count


class RosSubscriberSystem(LeafSystem):
    """Subscribes to a message, output to port.

    Warning:
        This may not given consistent results in a variable step simulation.
        To fix this:
        - unrestricted updates have to be bound in Python, or
        - as a more deterministic workaorund, a ZeroOrderHold can be placed
        afer this.
    """
    def __init__(self, topic, msg_type, period_sec):
        LeafSystem.__init__(self)
        self.topic = topic
        self.msg_type = msg_type
        self._sub = rospy.Subscriber(topic, msg_type, self._callback)
        self._lock = Lock()
        self._received_message = None
        self._received_message_count = 0
        # TODO(eric.cousineau): Use unrestricted update once it's exposed, or
        # add into a diagram with ZeroOrderHold.
        self._DeclareAbstractOutputPort(
            "msg", lambda: AbstractValue.Make(msg_type()),
            self._calc_message)

    def _callback(self, msg):
        with self._lock:
            self._received_message = msg
            self._received_message_count += 1

    def _calc_message(self, context, value):
        with self._lock:
            msg = self._received_message
        value.set_value(msg)

    def num_messages(self):
        """Number of messages received."""
        with self._lock:
            return self._received_message_count
