#!/usr/bin/env python

"""
analysis_sendiv.py
Analysis node for sensation (sensor measurements) divergence.
"""

__version__     = "0.0.1"
__author__      = "David Qiu"
__email__       = "dq@cs.cmu.edu"
__website__     = "http://mrsdprojects.ri.cmu.edu/2017teami/"
__copyright__   = "Copyright (C) 2017, the Moon Wreckers. All rights reserved."

import math, numpy, libMath
import csv
import rospy
from std_msgs.msg import String, Float32, UInt16
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3Stamped, Twist
from nav_msgs.msg import Odometry

analysis_rover = 'ak2'; # 'ak1', 'ak2'

spamwriter = None
csvfile = None

imu_data = None
wheelodom = None
ekfodom = None


def handleDataUpdatedEvent():
    if (imu_data == None or wheelodom == None or ekfodom == None):
        return;

    # source: imu
    imu_q = (
        imu_data.orientation.x,
        imu_data.orientation.y,
        imu_data.orientation.z,
        imu_data.orientation.w,
    )

    imu_thetaX, imu_thetaY, imu_thetaZ = libMath.quat_to_euler(imu_q)

    imu_omegaX = imu_data.angular_velocity.x
    imu_omegaY = imu_data.angular_velocity.y
    imu_omegaZ = imu_data.angular_velocity.z

    imu_ddx = imu_data.linear_acceleration.x
    imu_ddy = imu_data.linear_acceleration.y
    imu_ddz = imu_data.linear_acceleration.z

    # fusion: imu
    imu_ddx, imu_ddy, imu_ddz = libMath.quat_rotate_vector(imu_q, (imu_ddx, imu_ddy, imu_ddz))
    imu_ddx = imu_ddx + 0.2
    imu_ddy = imu_ddy + 0.4
    imu_ddz = imu_ddz + 0.4 + 9.81

    # source: odom
    odom_q = (
        wheelodom.pose.pose.orientation.w,
        wheelodom.pose.pose.orientation.x,
        wheelodom.pose.pose.orientation.y,
        wheelodom.pose.pose.orientation.z,
    )

    odom_thetaX, odom_thetaY, odom_thetaZ = libMath.quat_to_euler(odom_q)

    odom_omegaX = wheelodom.twist.twist.angular.x
    odom_omegaY = wheelodom.twist.twist.angular.y
    odom_omegaZ = wheelodom.twist.twist.angular.z

    odom_x = wheelodom.pose.pose.position.x
    odom_y = wheelodom.pose.pose.position.y
    odom_z = wheelodom.pose.pose.position.z

    odom_dx = wheelodom.twist.twist.linear.x
    odom_dy = wheelodom.twist.twist.linear.y
    odom_dz = wheelodom.twist.twist.linear.z

    # source: ekfodom
    ekfodom_q = (
        ekfodom.pose.pose.orientation.w,
        ekfodom.pose.pose.orientation.x,
        ekfodom.pose.pose.orientation.y,
        ekfodom.pose.pose.orientation.z
    )

    ekfodom_thetaX, ekfodom_thetaY, ekfodom_thetaZ = libMath.quat_to_euler(ekfodom_q)

    ekfodom_omegaX = ekfodom.twist.twist.angular.x
    ekfodom_omegaY = ekfodom.twist.twist.angular.y
    ekfodom_omegaZ = ekfodom.twist.twist.angular.z

    ekfodom_x = ekfodom.pose.pose.position.x
    ekfodom_y = ekfodom.pose.pose.position.y
    ekfodom_z = ekfodom.pose.pose.position.z

    ekfodom_dx = ekfodom.twist.twist.linear.x
    ekfodom_dy = ekfodom.twist.twist.linear.y
    ekfodom_dz = ekfodom.twist.twist.linear.z

    cur_time = rospy.get_time()

    cur_data = [
        cur_time,
        #imu_q[0], imu_q[1], imu_q[2], imu_q[3],
        imu_thetaX, imu_thetaY, imu_thetaZ,
        #imu_omegaX, imu_omegaY, imu_omegaZ,
        imu_ddx, imu_ddy, imu_ddz,
        #odom_q[0], odom_q[1], odom_q[2], odom_q[3],
        #odom_thetaX, odom_thetaY, odom_thetaZ,
        #odom_omegaX, odom_omegaY, odom_omegaZ,
        #odom_x, odom_y, odom_z,
        #odom_dx, odom_dy, odom_dz,
        #ekfodom_q[0], ekfodom_q[1], ekfodom_q[2], ekfodom_q[3],
        #ekfodom_thetaX, ekfodom_thetaY, ekfodom_thetaZ,
        #ekfodom_omegaX, ekfodom_omegaY, ekfodom_omegaZ,
        #ekfodom_x, ekfodom_y, ekfodom_z,
        #ekfodom_dx, ekfodom_dy, ekfodom_dz
    ]

    rospy.loginfo(cur_data)
    spamwriter.writerow(cur_data)


def cb_imu_data(data):
    global imu_data
    imu_data = data
    handleDataUpdatedEvent()


def cb_wheelodom(data):
    global wheelodom
    wheelodom = data
    handleDataUpdatedEvent()


def cb_ekfodom(data):
    global ekfodom
    ekfodom = data
    handleDataUpdatedEvent()


def analysis_sendiv():
    rospy.init_node('analysis_sendiv', anonymous=True)

    global csvfile
    global spamwriter
    csvfile = open('analysis_sensation_divergence.csv', 'wb')
    spamwriter = csv.writer(
        csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

    if analysis_rover == 'ak1':
        rospy.Subscriber("/ak1/imu/data", Imu, cb_imu_data)
        rospy.Subscriber("/ak1/odom", Odometry, cb_wheelodom)
        rospy.Subscriber("/ak1/odometry/filtered", Odometry, cb_ekfodom)
    elif analysis_rover == 'ak2':
        rospy.Subscriber("/ak2/imu/data", Imu, cb_imu_data)
        rospy.Subscriber("/ak2/odom", Odometry, cb_wheelodom)
        rospy.Subscriber("/ak2/odometry/filtered", Odometry, cb_ekfodom)
    else:
        rospy.logerror('invalid rover id: ' + analysis_rover)
        return

    rospy.spin() # use spin only in pure event handling programming model


if __name__ == '__main__':
    try:
        analysis_sendiv()
    except rospy.ROSInterruptException:
        pass
