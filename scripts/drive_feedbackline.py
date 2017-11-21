#!/usr/bin/env python

"""
drive_feedbackline.py
Drive with feedback along a straight line in a certain distance.
"""

__version__     = "0.0.1"
__author__      = "David Qiu"
__email__       = "dq@cs.cmu.edu"
__website__     = "http://mrsdprojects.ri.cmu.edu/2017teami/"
__copyright__   = "Copyright (C) 2017, the Moon Wreckers. All rights reserved."

import rospy
import math
import libMath
from std_msgs.msg import String, Float32, UInt16
from sensor_msgs.msg import Imu, JointState, Joy
from geometry_msgs.msg import Vector3Stamped, Twist
from nav_msgs.msg import Odometry

drive_control = ['ak1'] # 'ak1', 'ak2'
drive_dist = 0.5 # unit: m
drive_dist_overshoot = 0.05 # unit: m
drive_speed = 0.2 # unit: m/s
drive_start_time = 5.0 # unit: s

ak1_ekfodom = Odometry()
ak2_ekfodom = Odometry()


def norm2d(v1, v2):
    dd = (v1[0] - v2[0]) * (v1[0] - v2[0]) + (v1[1] - v2[1]) * (v1[1] - v2[1])
    d = math.sqrt(dd)
    return d


def cb_ak1_ekfodom(data):
    global ak1_ekfodom
    ak1_ekfodom = data


def cb_ak2_ekfodom(data):
    global ak2_ekfodom
    ak2_ekfodom = data


def drive_feedbackline():
    rospy.init_node('drive_feedbackline', anonymous=True)

    rospy.Subscriber('/ak1/odometry/filtered', Odometry, cb_ak1_ekfodom)
    rospy.Subscriber('/ak2/odometry/filtered', Odometry, cb_ak2_ekfodom)

    pub_ak1_cmdvel = rospy.Publisher('/ak1/cmd_vel', Twist, queue_size=10)
    pub_ak2_cmdvel = rospy.Publisher('/ak2/cmd_vel', Twist, queue_size=10)

    system_start_time = rospy.get_time()

    ak1_drive_status = 'stop' # 'stop', 'move'
    ak1_p0 = None

    ak2_drive_status = 'stop' # 'stop', 'move'
    ak2_p0 = None

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if 'ak1' in drive_control:
            x = ak1_ekfodom.pose.pose.position.x
            y = ak1_ekfodom.pose.pose.position.y
            z = 0
            q_w = ak1_ekfodom.pose.pose.orientation.w
            q_x = 0
            q_y = 0
            q_z = ak1_ekfodom.pose.pose.orientation.z
            theta_x, theta_y, theta_z = libMath.quat_to_euler((q_w, q_x, q_y, q_z))
            v_x = ak1_ekfodom.twist.twist.linear.x
            v_y = ak1_ekfodom.twist.twist.linear.y
            v_z = 0
            omega_x = 0
            omega_y = 0
            omega_z = ak1_ekfodom.twist.twist.angular.z

            cmd_vel = Twist()
            if rospy.get_time() - system_start_time < drive_start_time:
                ak1_drive_status = 'stop'
                ak1_p0 = (x, y)
            elif norm2d(ak1_p0, (x, y)) >= drive_dist - drive_dist_overshoot:
                ak1_drive_status = 'stop'
            else:
                cmd_vel.linear.x = drive_speed
                ak1_drive_status = 'move'

            pub_ak1_cmdvel.publish(cmd_vel)

            rospy.loginfo('AK1 \n' +
                          '- status="%s" delta=%f \n' +
                          '- p0=(%f, %f) \n' +
                          '- p=(%f,%f) v=(%f,%f) \n' +
                          '- theta_z=%f omega_z=%f \n' +
                          '- cmd_vel v_x=%f omega_z=%f',
                          ak1_drive_status, norm2d(ak1_p0, (x, y)) - drive_dist,
                          ak1_p0[0], ak1_p0[1],
                          x, y, v_x, v_y,
                          theta_z, omega_z,
                          cmd_vel.linear.x, cmd_vel.angular.z)

        rate.sleep()

    #rospy.spin() # use spin only in pure event handling programming model


if __name__ == '__main__':
    try:
        drive_feedbackline()
    except rospy.ROSInterruptException:
        pass
