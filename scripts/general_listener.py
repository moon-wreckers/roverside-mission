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
from sensor_msgs.msg import Imu, JointState, Joy
from geometry_msgs.msg import Vector3Stamped, Twist
from nav_msgs.msg import Odometry

log_control = [
    'ak1_towing_winch_status',
    'ak2_towing_winch_status',
    'ak1_towing_claw_status',
    'ak2_towing_claw_status',
    'ak1_towing_claw_actuator',
    'ak2_towing_claw_actuator',
    'ak1_towing_claw_pressure',
    'ak2_towing_claw_pressure'
]

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
ak1_jointstates = JointState()
ak2_jointstates = JointState()
ak1_wheelodom = Odometry()
ak2_wheelodom = Odometry()
ak1_ekfodom = Odometry()
ak2_ekfodom = Odometry()
ak1_towing_winch_status = UInt16()
ak2_towing_winch_status = UInt16()
ak1_towing_claw_status = UInt16()
ak2_towing_claw_status = UInt16()
ak1_towing_claw_actuator = UInt16()
ak2_towing_claw_actuator = UInt16()
ak1_towing_claw_pressure = UInt16()
ak2_towing_claw_pressure = UInt16()
ak1_joy = Joy()
ak2_joy = Joy()


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


def cb_ak1_jointstates(data):
    global ak1_jointstates
    ak1_jointstates = data


def cb_ak2_jointstates(data):
    global ak2_jointstates
    ak2_jointstates = data


def cb_ak1_wheelodom(data):
    global ak1_wheelodom
    ak1_wheelodom = data


def cb_ak2_wheelodom(data):
    global ak2_wheelodom
    ak2_wheelodom = data


def cb_ak1_ekfodom(data):
    global ak1_ekfodom
    ak1_ekfodom = data


def cb_ak2_ekfodom(data):
    global ak2_ekfodom
    ak2_ekfodom = data


def cb_ak1_towing_winch_status(data):
    global ak1_towing_winch_status
    ak1_towing_winch_status = data


def cb_ak2_towing_winch_status(data):
    global ak2_towing_winch_status
    ak2_towing_winch_status = data


def cb_ak1_towing_claw_status(data):
    global ak1_towing_claw_status
    ak1_towing_claw_status = data


def cb_ak2_towing_claw_status(data):
    global ak2_towing_claw_status
    ak2_towing_claw_status = data


def cb_ak1_towing_claw_actuator(data):
    global ak1_towing_claw_actuator
    ak1_towing_claw_actuator = data


def cb_ak2_towing_claw_actuator(data):
    global ak2_towing_claw_actuator
    ak2_towing_claw_actuator = data


def cb_ak1_towing_claw_pressure(data):
    global ak1_towing_claw_pressure
    ak1_towing_claw_pressure = data


def cb_ak2_towing_claw_pressure(data):
    global ak2_towing_claw_pressure
    ak2_towing_claw_pressure = data


def cb_ak1_joy(data):
    global ak1_joy
    ak1_joy = data


def cb_ak2_joy(data):
    global ak2_joy
    ak2_joy = data


def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/ak1/imu/data', Imu, cb_ak1_imu_data)
    rospy.Subscriber('/ak2/imu/data', Imu, cb_ak2_imu_data)
    rospy.Subscriber('/ak1/imu/rpy', Vector3Stamped, cb_ak1_imu_rpy)
    rospy.Subscriber('/ak2/imu/rpy', Vector3Stamped, cb_ak2_imu_rpy)
    rospy.Subscriber('/ak1/imu/mag', Vector3Stamped, cb_ak1_imu_mag)
    rospy.Subscriber('/ak2/imu/mag', Vector3Stamped, cb_ak2_imu_mag)
    rospy.Subscriber('/ak1/imu/temperature', Float32, cb_ak1_imu_temperature)
    rospy.Subscriber('/ak2/imu/temperature', Float32, cb_ak2_imu_temperature)
    rospy.Subscriber('/ak1/diagnostics/battery_voltage', Float32, cb_ak1_diagnostics_battery)
    rospy.Subscriber('/ak2/diagnostics/battery_voltage', Float32, cb_ak2_diagnostics_battery)
    rospy.Subscriber('/ak1/joint_states', JointState, cb_ak1_jointstates)
    rospy.Subscriber('/ak2/joint_states', JointState, cb_ak2_jointstates)
    rospy.Subscriber('/ak1/odom', Odometry, cb_ak1_wheelodom)
    rospy.Subscriber('/ak2/odom', Odometry, cb_ak2_wheelodom)
    rospy.Subscriber('/ak1/odometry/filtered', Odometry, cb_ak1_ekfodom)
    rospy.Subscriber('/ak2/odometry/filtered', Odometry, cb_ak2_ekfodom)
    rospy.Subscriber('/ak1/towing/winch/status', UInt16, cb_ak1_towing_winch_status)
    rospy.Subscriber('/ak2/towing/winch/status', UInt16, cb_ak2_towing_winch_status)
    rospy.Subscriber('/ak1/towing/claw/status', UInt16, cb_ak1_towing_claw_status)
    rospy.Subscriber('/ak2/towing/claw/status', UInt16, cb_ak2_towing_claw_status)
    rospy.Subscriber('/ak1/towing/claw/actuator', UInt16, cb_ak1_towing_claw_actuator)
    rospy.Subscriber('/ak2/towing/claw/actuator', UInt16, cb_ak2_towing_claw_actuator)
    rospy.Subscriber('/ak1/towing/claw/pressure', UInt16, cb_ak1_towing_claw_pressure)
    rospy.Subscriber('/ak2/towing/claw/pressure', UInt16, cb_ak2_towing_claw_pressure)
    rospy.Subscriber('/ak1/joy', Joy, cb_ak1_joy)
    rospy.Subscriber('/ak2/joy', Joy, cb_ak2_joy)

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

        if 'ak1_jointstates' in log_control:
            logmsg = 'AK1 joint_states:\n'
            for i in range(len(ak1_jointstates.name)):
                logmsg += '- ' + ak1_jointstates.name[i] + ': p=' + str(ak1_jointstates.position[i]) + ' v=' + str(ak1_jointstates.velocity[i]) + '\n'
            rospy.loginfo(logmsg)

        if 'ak2_jointstates' in log_control:
            logmsg = 'AK2 joint_states:\n'
            for i in range(len(ak2_jointstates.name)):
                logmsg += '- ' + ak2_jointstates.name[i] + ': p=' + str(ak2_jointstates.position[i]) + ' v=' + str(ak2_jointstates.velocity[i]) + '\n'
            rospy.loginfo(logmsg)

        if 'ak1_wheelodom' in log_control:
            rospy.loginfo('AK1 odom p=(%f,%f) qzw=(%f,%f) v=(%f,%f) wz=%f',
                ak1_wheelodom.pose.pose.position.x,
                ak1_wheelodom.pose.pose.position.y,
                ak1_wheelodom.pose.pose.orientation.z,
                ak1_wheelodom.pose.pose.orientation.w,
                ak1_wheelodom.twist.twist.linear.x,
                ak1_wheelodom.twist.twist.linear.y,
                ak1_wheelodom.twist.twist.angular.z)

        if 'ak2_wheelodom' in log_control:
            rospy.loginfo('AK2 odom p=(%f,%f) qzw=(%f,%f) v=(%f,%f) wz=%f',
                ak2_wheelodom.pose.pose.position.x,
                ak2_wheelodom.pose.pose.position.y,
                ak2_wheelodom.pose.pose.orientation.z,
                ak2_wheelodom.pose.pose.orientation.w,
                ak2_wheelodom.twist.twist.linear.x,
                ak2_wheelodom.twist.twist.linear.y,
                ak2_wheelodom.twist.twist.angular.z)

        if 'ak1_ekfodom' in log_control:
            rospy.loginfo('AK1 ekfodom p=(%f,%f) qzw=(%f,%f) v=(%f,%f) wz=%f',
                ak1_ekfodom.pose.pose.position.x,
                ak1_ekfodom.pose.pose.position.y,
                ak1_ekfodom.pose.pose.orientation.z,
                ak1_ekfodom.pose.pose.orientation.w,
                ak1_ekfodom.twist.twist.linear.x,
                ak1_ekfodom.twist.twist.linear.y,
                ak1_ekfodom.twist.twist.angular.z)

        if 'ak2_ekfodom' in log_control:
            rospy.loginfo('AK2 ekfodom p=(%f,%f) qzw=(%f,%f) v=(%f,%f) wz=%f',
                ak2_ekfodom.pose.pose.position.x,
                ak2_ekfodom.pose.pose.position.y,
                ak2_ekfodom.pose.pose.orientation.z,
                ak2_ekfodom.pose.pose.orientation.w,
                ak2_ekfodom.twist.twist.linear.x,
                ak2_ekfodom.twist.twist.linear.y,
                ak2_ekfodom.twist.twist.angular.z)

        if 'ak1_towing_winch_status' in log_control:
            rospy.loginfo('AK1 towing_winch_status=%d', ak1_towing_winch_status.data)

        if 'ak2_towing_winch_status' in log_control:
            rospy.loginfo('AK2 towing_winch_status=%d', ak2_towing_winch_status.data)

        if 'ak1_towing_claw_status' in log_control:
            rospy.loginfo('AK1 towing_claw_status=%d', ak1_towing_claw_status.data)

        if 'ak2_towing_claw_status' in log_control:
            rospy.loginfo('AK2 towing_claw_status=%d', ak2_towing_claw_status.data)

        if 'ak1_towing_claw_actuator' in log_control:
            rospy.loginfo('AK1 towing_claw_actuator=%d', ak1_towing_claw_actuator.data)

        if 'ak2_towing_claw_actuator' in log_control:
            rospy.loginfo('AK2 towing_claw_actuator=%d', ak2_towing_claw_actuator.data)

        if 'ak1_towing_claw_pressure' in log_control:
            rospy.loginfo('AK1 towing_claw_pressure=%d', ak1_towing_claw_pressure.data)

        if 'ak2_towing_claw_pressure' in log_control:
            rospy.loginfo('AK2 towing_claw_pressure=%d', ak2_towing_claw_pressure.data)

        if 'ak1_joy' in log_control:
            logmsg = 'AK1 joy axes = '
            logmsg += '['
            for i_axis in range(len(ak1_joy.axes)):
                logmsg += str(ak1_joy.axes[i_axis]) + ','
            logmsg += ']'
            rospy.loginfo(logmsg)

            logmsg = 'AK1 joy buttons = '
            logmsg += '['
            for i_button in range(len(ak1_joy.buttons)):
                logmsg += str(ak1_joy.buttons[i_button]) + ','
            logmsg += ']'
            rospy.loginfo(logmsg)

        if 'ak2_joy' in log_control:
            logmsg = 'AK2 joy axes = '
            logmsg += '['
            for i_axis in range(len(ak2_joy.axes)):
                logmsg += str(ak2_joy.axes[i_axis]) + ','
            logmsg += ']'
            rospy.loginfo(logmsg)

            logmsg = 'AK2 joy buttons = '
            logmsg += '['
            for i_button in range(len(ak2_joy.buttons)):
                logmsg += str(ak2_joy.buttons[i_button]) + ','
            logmsg += ']'
            rospy.loginfo(logmsg)


        rate.sleep()

    #rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
