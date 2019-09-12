#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


publisher = rospy.Publisher('range_ahead', Float32, queue_size=1)


def subscriber_callback(scan):
    f = Float32()
    f.data = min(scan.ranges)
    publisher.publish(f)


rospy.init_node('range_ahead_node')
subscriber = rospy.Subscriber('scan', LaserScan, subscriber_callback, queue_size=1)

rospy.spin()
