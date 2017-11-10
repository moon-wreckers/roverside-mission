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

import rospy
from std_msgs.msg import String, Float32, UInt16
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry, Twist

ak1_imu_data = Imu()
ak2_imu_data = Imu()
ak1_wheelodom = Odometry()
ak2_wheelodom = Odometry()


def cb_ak1_imu_data(data):
    global ak1_imu_data
    ak1_imu_data = data


def cb_ak2_imu_data(data):
    global ak2_imu_data
    ak2_imu_data = data


def cb_ak1_wheelodom(data):
    global ak1_wheelodom
    ak1_wheelodom = data


def cb_ak2_wheelodom(data):
    global ak2_wheelodom
    ak2_wheelodom = data


def analysis_sendiv():
    rospy.init_node('analysis_sendiv', anonymous=True)

    rospy.Subscriber("/ak1/imu/data", Imu, cb_ak1_imu_data)
    rospy.Subscriber("/ak2/imu/data", Imu, cb_ak2_imu_data)
    rospy.Subscriber("/ak1/odom", Odometry, cb_ak1_wheelodom)
    rospy.Subscriber("/ak2/odom", Odometry, cb_ak2_wheelodom)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo('AK1 IMU q=(%f,%f,%f,%f) w=(%f,%f,%f) a=(%f,%f,%f)',
            ak1_imu_data.orientation.x,
            ak1_imu_data.orientation.y,
            ak1_imu_data.orientation.z,
            ak1_imu_data.orientation.w,
            ak1_imu_data.angular_velocity.x,
            ak1_imu_data.angular_velocity.y,
            ak1_imu_data.angular_velocity.z,
            ak1_imu_data.linear_acceleration.x,
            ak1_imu_data.linear_acceleration.y,
            ak1_imu_data.linear_acceleration.z)

        rospy.loginfo('AK2 IMU q=(%f,%f,%f,%f) w=(%f,%f,%f) a=(%f,%f,%f)',
            ak2_imu_data.orientation.x,
            ak2_imu_data.orientation.y,
            ak2_imu_data.orientation.z,
            ak2_imu_data.orientation.w,
            ak2_imu_data.angular_velocity.x,
            ak2_imu_data.angular_velocity.y,
            ak2_imu_data.angular_velocity.z,
            ak2_imu_data.linear_acceleration.x,
            ak2_imu_data.linear_acceleration.y,
            ak2_imu_data.linear_acceleration.z)

        rospy.loginfo('AK1 odom p=(%f,%f) qzw=(%f,%f) v=(%f,%f) wz=%f',
            ak1_wheelodom.pose.pose.position.x,
            ak1_wheelodom.pose.pose.position.y,
            ak1_wheelodom.pose.pose.orientation.z,
            ak1_wheelodom.pose.pose.orientation.w,
            ak1_wheelodom.twist.twist.linear.x,
            ak1_wheelodom.twist.twist.linear.y,
            ak1_wheelodom.twist.twist.angular.z)

        rospy.loginfo('AK2 odom p=(%f,%f) qzw=(%f,%f) v=(%f,%f) wz=%f',
            ak2_wheelodom.pose.pose.position.x,
            ak2_wheelodom.pose.pose.position.y,
            ak2_wheelodom.pose.pose.orientation.z,
            ak2_wheelodom.pose.pose.orientation.w,
            ak2_wheelodom.twist.twist.linear.x,
            ak2_wheelodom.twist.twist.linear.y,
            ak2_wheelodom.twist.twist.angular.z)

        rate.sleep()

    #rospy.spin() # use spin only in pure event handling programming model


if __name__ == '__main__':
    try:
        analysis_sendiv()
    except rospy.ROSInterruptException:
        pass
