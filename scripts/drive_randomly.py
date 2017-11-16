#!/usr/bin/env python

"""
drive_randomly.py
A node that drives the rovers randomly.
"""

__version__     = "0.0.1"
__author__      = "David Qiu"
__email__       = "dq@cs.cmu.edu"
__website__     = "http://mrsdprojects.ri.cmu.edu/2017teami/"
__copyright__   = "Copyright (C) 2017, the Moon Wreckers. All rights reserved."

import rospy
from std_msgs.msg import String, Float32, UInt16
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3Stamped, Twist
from nav_msgs.msg import Odometry
import random

drive_control = ['ak2'] # 'ak1', 'ak2'
drive_round_min = 1.0 # sec
drive_round_max = 5.0 # sec
drive_linear_min = -0.75 # ratio
drive_linear_max = 0.75 # ratio
drive_angular_min = -0.35 # ratio
drive_angular_max = 0.35 # ratio


def get_random_cmd():
    cmd_round = random.uniform(drive_round_min, drive_round_max)
    cmd_linear = random.uniform(drive_linear_min, drive_linear_max)
    cmd_angular = random.uniform(drive_angular_min, drive_angular_max)
    return (cmd_round, cmd_linear, cmd_angular)

def drive_randomly():
    rospy.init_node('drive_randomly', anonymous=True)

    pub_ak1_cmdvel = rospy.Publisher('/ak1/cmd_vel', Twist, queue_size=10)
    pub_ak2_cmdvel = rospy.Publisher('/ak2/cmd_vel', Twist, queue_size=10)

    last_time = 0
    last_round = 0
    last_linear = 0
    last_angular = 0

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cmd_vel = Twist()

        if rospy.get_time() > last_time + last_round:
            last_time = rospy.get_time()
            last_round, last_linear, last_angular = get_random_cmd()

        cmd_vel.linear.x = last_linear
        cmd_vel.angular.z = last_angular

        logmsg = 'cmd_vel: t=%f v=(%f,%f,%f) w=(%f,%f,%f) >>' % (
                    last_time + last_round - rospy.get_time(),
                    cmd_vel.linear.x,
                    cmd_vel.linear.y,
                    cmd_vel.linear.z,
                    cmd_vel.angular.x,
                    cmd_vel.angular.y,
                    cmd_vel.angular.z)

        if 'ak1' in drive_control:
            logmsg += ' ak1'
            pub_ak1_cmdvel.publish(cmd_vel);

        if 'ak2' in drive_control:
            logmsg += ' ak2'
            pub_ak2_cmdvel.publish(cmd_vel);

        rospy.loginfo(logmsg)

        rate.sleep()

    #rospy.spin()


if __name__ == '__main__':
    try:
        drive_randomly()
    except rospy.ROSInterruptException:
        pass
