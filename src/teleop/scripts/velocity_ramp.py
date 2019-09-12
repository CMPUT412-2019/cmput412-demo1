#!/usr/bin/env python

from time import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist


def normalize(a):
    norm = np.linalg.norm(a)
    if np.allclose(norm, 0):
        return np.zeros_like(a)
    else:
        return a / norm


class Ramp:
    def __init__(self, acceleration_norm, shape):
        self.a_norm = acceleration_norm
        self.current = np.zeros(shape=shape)
        self.target = np.zeros(shape=shape)
        self.prev_time = None

    def set_target(self, target):
        self.target = target

    def update(self):
        if self.prev_time is None:
            self.prev_time = time()
            return
        elapsed_time = time() - self.prev_time
        self.prev_time = time()
        v_norm = elapsed_time * self.a_norm
        if np.linalg.norm(self.target - self.current) < v_norm:
            self.current = self.target
        else:
            self.current += v_norm * normalize(self.target - self.current)

    def get(self):
        return self.current


class VelocityRamp:
    def __init__(self, linear_accel, angular_accel):
        self.linear = Ramp(linear_accel, (3,))
        self.angular = Ramp(angular_accel, (3,))

    def set_targets(self, linear_target, angular_target):
        self.linear.set_target(linear_target)
        self.angular.set_target(angular_target)

    def update(self):
        self.linear.update()
        self.angular.update()

    def subscriber_callback(self, twist):
        linear = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        angular = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
        self.set_targets(linear, angular)


def main():
    rospy.init_node('velocity_ramp_node')
    velocity_ramp = VelocityRamp(
        linear_accel=rospy.get_param('~linear_accel', 1),
        angular_accel=rospy.get_param('~angular_accel', 1),
    )
    publisher = rospy.Publisher('cmd_vel_out', Twist, queue_size=1)
    subscriber = rospy.Subscriber('cmd_vel_in', Twist, velocity_ramp.subscriber_callback, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        velocity_ramp.update()
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = velocity_ramp.linear.get()
        t.angular.x, t.angular.y, t.angular.z = velocity_ramp.angular.get()
        publisher.publish(t)
        rate.sleep()


if __name__ == '__main__':
    main()
