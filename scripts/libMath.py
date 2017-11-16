#!/usr/bin/env python

"""
libMath.py
Assistant mathematics library.
"""

__version__     = "0.0.1"
__author__      = "David Qiu"
__email__       = "dq@cs.cmu.edu"
__website__     = "http://mrsdprojects.ri.cmu.edu/2017teami/"
__copyright__   = "Copyright (C) 2017, the Moon Wreckers. All rights reserved."

import math
import numpy


def normalize(v, tolerance=0.00001):
    """
    Normalize a vector.

    @param The original vector as a tuple.
    @param tolerance The numerical precision tolerance.
    @return The normalized vector as a tuple.
    """

    mag2 = numpy.sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = numpy.sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v


def quat_multiply(q1, q2):
    """
    Multiple two quaternions.

    @param q1 The left operand quaternion as a tuple.
    @param q2 The right operand quaternion as a tuple.
    @return The product of the two quaternions as a tuple.
    """

    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

    return w, x, y, z


def quat_conjugate(q):
    """
    Calculate the conjugate of a quaternion.

    @param q The original quaternion as a tuple.
    @return The conjugate quaternion as a tuple.
    """

    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_rotate_vector(q, v):
    """
    Rotate a 3D vector by a quaternion.

    @param q The quaternion as a tuple representing the rotation.
    @param v The vector as a tuple to rotate.
    @return The rotated vector as a tuple.
    """

    qv = (0.0,) + v
    return quat_multiply(quat_multiply(q, qv), quat_conjugate(q))[1:]


def axisangle_to_quat(v, theta):
    """
    Convert an axis-angle rotation representation to a quaternion rotation
    representation.

    @param v The axis to rotate about.
    @param theta The angle to rotate by.
    @return The quaternion representation of the rotation.
    """

    v = normalize(v)
    x, y, z = v
    theta /= 2

    w = math.cos(theta)
    x = x * math.sin(theta)
    y = y * math.sin(theta)
    z = z * math.sin(theta)

    return w, x, y, z


def quat_to_axisangle(q):
    """
    Convert a quaternion rotation representation to an axis-angle rotation
    representation.

    @param The quaternion representation of the rotation.
    @return v The axis to rotate about.
    @return theta The angle to rotate by.
    """

    w, v = q[0], q[1:]
    theta = math.acos(w) * 2.0
    return normalize(v), theta


if __name__ == "__main__":
    # test quaternion operations
    x_axis_unit = (1, 0, 0)
    y_axis_unit = (0, 1, 0)
    z_axis_unit = (0, 0, 1)

    r1 = axisangle_to_quat(x_axis_unit, 3.14 / 2)
    (axis_r1, angle_r1) = quat_to_axisangle(r1)
    r1 = axisangle_to_quat(axis_r1, angle_r1)
    r2 = axisangle_to_quat(y_axis_unit, 3.14 / 2)
    r3 = axisangle_to_quat(z_axis_unit, 3.14 / 2)

    v = quat_rotate_vector(r1, y_axis_unit)
    v = quat_rotate_vector(r2, v)
    v = quat_rotate_vector(r3, v)

    print(v)
