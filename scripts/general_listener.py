#!/usr/bin/env python

"""
general_listener.py
A general listener that subscribe to all relevant topics.
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

log_control = ['ak1_imu_data', 'ak2_imu_data']

ak1_imu_data = Imu()
ak2_imu_data = Imu()
ak1_imu_rpy = Vector3Stamped()
ak2_imu_rpy = Vector3Stamped()
ak1_imu_mag = Vector3Stamped()
ak2_imu_mag = Vector3Stamped()
ak1_imu_temperature = Float32()
ak2_imu_temperature = Float32()
ak1_diagnostics_battery = Float32()
ak2_diagnostics_battery = Float32()


def cb_ak1_imu_data(data):
    global ak1_imu_data
    ak1_imu_data = data


def cb_ak2_imu_data(data):
    global ak2_imu_data
    ak2_imu_data = data


def cb_ak1_imu_rpy(data):
    global ak1_imu_rpy
    ak1_imu_rpy = data


def cb_ak2_imu_rpy(data):
    global ak2_imu_rpy
    ak2_imu_rpy = data


def cb_ak1_imu_mag(data):
    global ak1_imu_mag
    ak1_imu_mag = data


def cb_ak2_imu_mag(data):
    global ak2_imu_mag
    ak2_imu_mag = data


def cb_ak1_imu_temperature(data):
    global ak1_imu_temperature
    ak1_imu_temperature = data


def cb_ak2_imu_temperature(data):
    global ak2_imu_temperature
    ak2_imu_temperature = data


def cb_ak1_diagnostics_battery(data):
    global ak1_diagnostics_battery
    ak1_diagnostics_battery = data


def cb_ak2_diagnostics_battery(data):
    global ak2_diagnostics_battery
    ak2_diagnostics_battery = data


def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/ak1/imu/data", Imu, cb_ak1_imu_data)
    rospy.Subscriber("/ak2/imu/data", Imu, cb_ak2_imu_data)
    rospy.Subscriber("/ak1/imu/rpy", Vector3Stamped, cb_ak1_imu_rpy)
    rospy.Subscriber("/ak2/imu/rpy", Vector3Stamped, cb_ak2_imu_rpy)
    rospy.Subscriber("/ak1/imu/mag", Vector3Stamped, cb_ak1_imu_mag)
    rospy.Subscriber("/ak2/imu/mag", Vector3Stamped, cb_ak2_imu_mag)
    rospy.Subscriber("/ak1/imu/temperature", Float32, cb_ak1_imu_temperature)
    rospy.Subscriber("/ak2/imu/temperature", Float32, cb_ak2_imu_temperature)
    rospy.Subscriber("/ak1/diagnostics/battery_voltage", Float32, cb_ak1_diagnostics_battery)
    rospy.Subscriber("/ak2/diagnostics/battery_voltage", Float32, cb_ak2_diagnostics_battery)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if 'ak1_imu_data' in log_control:
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

        if 'ak2_imu_data' in log_control:
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

        if 'ak1_imu_rpy' in log_control:
            rospy.loginfo('AK1 IMU rpy=(%f,%f,%f)',
                ak1_imu_rpy.vector.x,
                ak1_imu_rpy.vector.y,
                ak1_imu_rpy.vector.z)

        if 'ak2_imu_rpy' in log_control:
            rospy.loginfo('AK2 IMU rpy=(%f,%f,%f)',
                ak2_imu_rpy.vector.x,
                ak2_imu_rpy.vector.y,
                ak2_imu_rpy.vector.z)

        if 'ak1_imu_mag' in log_control:
            rospy.loginfo('AK1 IMU mag=(%f,%f,%f)',
                ak1_imu_mag.vector.x,
                ak1_imu_mag.vector.y,
                ak1_imu_mag.vector.z)

        if 'ak2_imu_mag' in log_control:
            rospy.loginfo('AK2 IMU mag=(%f,%f,%f)',
                ak2_imu_mag.vector.x,
                ak2_imu_mag.vector.y,
                ak2_imu_mag.vector.z)

        if 'ak1_imu_temperature' in log_control:
            rospy.loginfo('AK1 IMU temperature=%f',
                ak1_imu_temperature.data)

        if 'ak2_imu_temperature' in log_control:
            rospy.loginfo('AK2 IMU temperature=%f',
                ak2_imu_temperature.data)

        if 'ak1_diagnostics_battery' in log_control:
            rospy.loginfo('AK1 IMU battery=%f',
                ak1_diagnostics_battery.data)

        if 'ak2_diagnostics_battery' in log_control:
            rospy.loginfo('AK2 IMU battery=%f',
                ak2_diagnostics_battery.data)

        rate.sleep()

    #rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
