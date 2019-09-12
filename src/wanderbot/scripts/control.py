#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from smach import State, Sequence
from smach_ros import IntrospectionServer
from std_msgs.msg import Float32


class DriveForwardState(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.publisher = rospy.Publisher('cmd_vel', Twist, latch=True, queue_size=10)
        self.subscriber = rospy.Subscriber('range_ahead', Float32, self.subscriber_callback, queue_size=1)
        self.range_ahead = 1

    def execute(self, userdata):
        timeout_time = rospy.get_rostime() + rospy.Duration(30)
        range_threshold = rospy.get_param('~range_threshold')
        cooldown_time = rospy.get_param('~cooldown_time')
        rate = rospy.Rate(10)

        t = Twist()
        t.linear.x = 1
        self.publisher.publish(t)

        status = None
        while True:
            if rospy.get_rostime() > timeout_time:
                status = 'succeeded'
                break
            if self.range_ahead < range_threshold:
                status = 'succeeded'
                break
            rate.sleep()

        self.publisher.publish(Twist())
        rospy.sleep(cooldown_time)
        return status

    def subscriber_callback(self, message):
        self.range_ahead = message.data


class TurnState(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.publisher = rospy.Publisher('cmd_vel', Twist, latch=True, queue_size=10)

    def execute(self, userdata):
        cooldown_time = rospy.get_param('~cooldown_time')
        t = Twist()
        t.angular.z = 1
        self.publisher.publish(t)
        rospy.sleep(rospy.Duration(2))
        self.publisher.publish(Twist())
        rospy.sleep(cooldown_time)
        return 'succeeded'


rospy.init_node('control_node')

sequence = Sequence(outcomes=['succeeded', 'aborted', 'preempted'], connector_outcome='succeeded')
with sequence:
    Sequence.add('FORWARD', DriveForwardState())
    Sequence.add('TURN', TurnState(), transitions={'succeeded': 'FORWARD'})

sis = IntrospectionServer('smach_server', sequence, '/SM_ROOT')
sis.start()

sequence.execute()
