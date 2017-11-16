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

import math
import rospy
from std_msgs.msg import String, Float32, UInt16
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3Stamped, Twist
from nav_msgs.msg import Odometry

analysis_rover = 'ak2'; # 'ak1', 'ak2'

imu_data = None
imu_data_last = None
wheelodom = None
ekfodom = None


def cb_imu_data(data):
    global imu_data
    global imu_data_last
    imu_data_last = imu_data
    imu_data = data


def cb_wheelodom(data):
    global wheelodom
    wheelodom = data


def cb_ekfodom(data):
    global ekfodom
    ekfodom = data


def quaternion_to_euler_angle(w, x, y, z):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X_rad = math.atan2(t0, t1)

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y_rad = math.asin(t2)

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z_rad = math.atan2(t3, t4)

	return X_rad, Y_rad, Z_rad


def analysis_sendiv():
    rospy.init_node('analysis_sendiv', anonymous=True)

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

    imu_dx = None
    imu_dy = None
    imu_dz = None

    start_time = rospy.get_time()
    last_time = None

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if (imu_data == None or
            imu_data_last == None or
            wheelodom == None or
            ekfodom == None):
            continue

        # imu
        imu_thetaX, imu_thetaY, imu_thetaZ = quaternion_to_euler_angle(
            imu_data.orientation.w,
            imu_data.orientation.x,
            imu_data.orientation.y,
            imu_data.orientation.z
        )

        imu_omegaX = imu_data.angular_velocity.x
        imu_omegaY = imu_data.angular_velocity.y
        imu_omegaZ = imu_data.angular_velocity.z

        imu_ddx = imu_data.linear_acceleration.x
        imu_ddy = imu_data.linear_acceleration.y
        imu_ddz = imu_data.linear_acceleration.z

        imu_ddx_last = imu_data_last.linear_acceleration.x
        imu_ddy_last = imu_data_last.linear_acceleration.y
        imu_ddz_last = imu_data_last.linear_acceleration.z

        # odom
        odom_thetaX, odom_thetaY, odom_thetaZ = quaternion_to_euler_angle(
            wheelodom.pose.pose.orientation.w,
            wheelodom.pose.pose.orientation.x,
            wheelodom.pose.pose.orientation.y,
            wheelodom.pose.pose.orientation.z
        )

        odom_omegaX = wheelodom.twist.twist.angular.x
        odom_omegaY = wheelodom.twist.twist.angular.y
        odom_omegaZ = wheelodom.twist.twist.angular.z

        odom_x = wheelodom.pose.pose.position.x
        odom_y = wheelodom.pose.pose.position.y
        odom_z = wheelodom.pose.pose.position.z

        odom_dx = wheelodom.twist.twist.linear.x
        odom_dy = wheelodom.twist.twist.linear.y
        odom_dz = wheelodom.twist.twist.linear.z

        # ekfodom
        ekfodom_thetaX, ekfodom_thetaY, ekfodom_thetaZ = quaternion_to_euler_angle(
            ekfodom.pose.pose.orientation.w,
            ekfodom.pose.pose.orientation.x,
            ekfodom.pose.pose.orientation.y,
            ekfodom.pose.pose.orientation.z
        )

        ekfodom_omegaX = ekfodom.twist.twist.angular.x
        ekfodom_omegaY = ekfodom.twist.twist.angular.y
        ekfodom_omegaZ = ekfodom.twist.twist.angular.z

        ekfodom_x = ekfodom.pose.pose.position.x
        ekfodom_y = ekfodom.pose.pose.position.y
        ekfodom_z = ekfodom.pose.pose.position.z

        ekfodom_dx = ekfodom.twist.twist.linear.x
        ekfodom_dy = ekfodom.twist.twist.linear.y
        ekfodom_dz = ekfodom.twist.twist.linear.z

        # imu integral
        if imu_dx == None or imu_dy == None or imu_dz == None:
            imu_dx = odom_dx
            imu_dy = odom_dy
            imu_dz = odom_dz
        else:
            imu_dx += (rospy.get_time() - last_time) * (imu_ddx + imu_ddx_last) / 2.0
            imu_dy += (rospy.get_time() - last_time) * (imu_ddy + imu_ddy_last) / 2.0
            imu_dz += (rospy.get_time() - last_time) * (imu_ddz + imu_ddz_last) / 2.0

        # |odom - ekfodom|
        """
        logmsg = '|odom - ekfodom|:\n'
        logmsg += '- x: |(%f) - (%f)| = %f\n' % (odom_x, ekfodom_x, abs(odom_x - ekfodom_x))
        logmsg += '- y: |(%f) - (%f)| = %f\n' % (odom_y, ekfodom_y, abs(odom_y - ekfodom_y))
        logmsg += '- z: |(%f) - (%f)| = %f\n' % (odom_z, ekfodom_z, abs(odom_z - ekfodom_z))
        """

        # d_imu
        logmsg = 'd_imu:\n'
        logmsg += '- dx: %f\n' % (imu_dx)
        logmsg += '- dy: %f\n' % (imu_dy)
        logmsg += '- dz: %f\n' % (imu_dz)

        # |dd_odom - dd_imu|


        rospy.loginfo(logmsg)

        last_time = rospy.get_time()
        rate.sleep()

    #rospy.spin() # use spin only in pure event handling programming model


if __name__ == '__main__':
    try:
        analysis_sendiv()
    except rospy.ROSInterruptException:
        pass
