#!/usr/bin/env python

"""
drivebnf.py
A node that drives the rovers back and forth.
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

drive_control = ['ak1'] # 'ak1', 'ak2'
drive_round = 8.0 # sec, back and forth as one round
drive_velocity = 0.2 # ratio


def drivebnf():
    rospy.init_node('drivebnf', anonymous=True)

    pub_ak1_cmdvel = rospy.Publisher('/ak1/cmd_vel', Twist, queue_size=10)
    pub_ak2_cmdvel = rospy.Publisher('/ak2/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cmd_vel = Twist()
        if rospy.get_time() % drive_round < (drive_round / 2.0):
            cmd_vel.linear.x = drive_velocity
        else:
            cmd_vel.linear.x = -drive_velocity

        logmsg = 'cmd_vel: v=(%f,%f,%f) w=(%f,%f,%f) >>' % (
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
        drivebnf()
    except rospy.ROSInterruptException:
        pass
