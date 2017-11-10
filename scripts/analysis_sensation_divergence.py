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


def analysis_sendiv():
    rospy.init_node('analysis_sendiv', anonymous=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # you code here...

        rate.sleep()

    #rospy.spin() # use spin only in pure event handling programming model


if __name__ == '__main__':
    try:
        analysis_sendiv()
    except rospy.ROSInterruptException:
        pass
