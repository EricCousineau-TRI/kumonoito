#!/usr/bin/env python2

# TODO(eric.cousineau): Put this in a module that isn't `test`.

from collections import deque
import gc
import sys
import time
import unittest

import numpy as np
import rospy
import rostest
import std_msgs

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import AbstractValue, DiagramBuilder
from pydrake.systems.primitives import ZeroOrderHold

from drake_ros_systems import RosPublisherSystem, RosSubscriberSystem

PKG = 'kumonoito'
NAME = 'drake_ros_systems_test'


def sleep_for_sub():
    # ROS subscribers are not always ready after being created, or may not
    # immediately process messages after one is published.
    time.sleep(0.1)


class TestDrakeRosSystems(unittest.TestCase):
    def test_publisher(self):
        """Make a ROS publisher system, and ensure a direct ROS subscriber can
        listen to it."""
        topic = "/test_message"
        for i in range(10):
            # Create dummy message.
            msg = std_msgs.msg.Float64()
            msg.data = i
            pub_system = RosPublisherSystem(topic, type(msg), 0.1)
            # Add subscriber, wait.
            msgs_received = deque()  # Thread-safe.
            sub = rospy.Subscriber(topic, type(msg), msgs_received.append)
            sleep_for_sub()
            # Create and simulate.
            context = pub_system.CreateDefaultContext()
            context.FixInputPort(0, AbstractValue.Make(msg))
            simulator = Simulator(pub_system, context)
            simulator.StepTo(1.)
            # Wait, then check results.
            sleep_for_sub()
            self.assertEqual(pub_system.num_messages(), 10)
            self.assertEqual(pub_system.num_messages(), len(msgs_received))
            for msg_received in msgs_received:
                self.assertEqual(msg_received.data, msg.data)

    def test_subscriber(self):
        """Make a ROS subscriber system, and esnure a direct ROS publisher
        can publish to it."""
        topic = "/test_message"
        for i in range(10):
            msg = std_msgs.msg.Float64()
            pub = rospy.Publisher(topic, type(msg), queue_size=10)
            sub_system = RosSubscriberSystem(topic,  type(msg), 0.1)
            sleep_for_sub()
            zoh = ZeroOrderHold(0.1, AbstractValue.Make(msg))
            builder = DiagramBuilder()
            builder.AddSystem(sub_system)
            builder.AddSystem(zoh)
            builder.Connect(
                sub_system.get_output_port(0), zoh.get_input_port(0))
            diagram = builder.Build()
            context = diagram.CreateDefaultContext()
            zoh_context = diagram.GetMutableSubsystemContext(zoh, context)
            simulator = Simulator(diagram, context)
            t = 0.
            while t < 1.:
                # Use `eps` to ensure the ZOH will trigger.
                t += 0.1 + np.finfo(float).eps
                msg.data = i * 10 + t
                pub.publish(msg)
                time.sleep(0.01)  # We can sleep less here.
                simulator.StepTo(t)
                msg_zoh = zoh_context.get_abstract_state(0).get_value()
                self.assertEqual(msg_zoh.data, msg.data)
            self.assertEqual(sub_system.num_messages(), 10)


if __name__ == '__main__':
    # WARNING: Maybe something I (Eric) did here, but failure in this script 
    # can still cause `rostest` to exit with status 0...
    rospy.init_node(NAME, anonymous=True)
    rostest.rosrun(PKG, NAME, TestDrakeRosSystems, sys.argv)
