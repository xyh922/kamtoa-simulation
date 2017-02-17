#!/usr/bin/env python

import tf
from math import pi

import math

# from tf.transformations import *

from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose, Quaternion, Point
import numpy


def print_buff(buff):
    for i in buff:
        if type(i) is str:
            o = ord(i)
        else:
            o = i
        print "%02d" % o,
    print


def pprint(item, indent=0):

    space = indent * 2

    type_ = type(item)

    if type_ == list:
        indent += 1
        for i in item:
            pprint(i, indent)

        print (" " * space) + ("-" * 5)

    elif type_ == dict:
        for key, value in item.items():
            print (" " * space) + key
            pprint(value, indent + 1)

        print (" " * space) + "=" * 5

    else:  # type_ in [int, float, str]:
        print (" " * space) + str(item)


def to_list(a):  # a is twist

    type_ = type(a)

    if type_ == Twist:
        return to_list(a.linear) + to_list(a.angular)
    elif type_ in [Vector3, Point]:
        return [a.x, a.y, a.z]
    elif type_ == Quaternion:
        return [a.w, a.x, a.y, a.z]

    elif type_ == PoseStamped:
        return [a.header, to_list(a.pose)]

    elif type_ == Pose:
        return [to_list(a.position), to_list(a.orientation)]

    else:
        print a
        raise ValueError('cannot to_list for ', type_)


def ttl(a):  # a is twist
    tmp = [0, 0, 0, 0, 0, 0]
    tmp[0] = a.linear.x
    tmp[1] = a.linear.y
    tmp[2] = a.linear.z
    tmp[3] = a.angular.x
    tmp[4] = a.angular.y
    tmp[5] = a.angular.z
    return tmp


def ltt(a):  # a is list
    tmp = Twist()
    tmp.linear.x = a[0]
    tmp.linear.y = a[1]
    tmp.linear.z = a[2]
    tmp.angular.x = a[3]
    tmp.angular.y = a[4]
    tmp.angular.z = a[5]
    return tmp


def normalize_angle(ang):
    return (ang + 4 * pi) % (2 * pi)


def diffAng(a, b):

    a, b = [normalize_angle(angle) for angle in [a, b]]

    diff = a - b

    if diff > pi:
        diff -= 2 * pi
    elif diff < -pi:
        diff += 2 * pi

    return diff


def get_yaw_from_direction(l):

    x, y, z = l

    return normalize_angle(math.atan2(y, x))


def quaternion_from_euler(r, p, y):

    w, x, y, z = tf.transformations.quaternion_from_euler(r, p, y)

    return Quaternion(w, x, y, z)

print quaternion_from_euler(0, 0, math.pi)


def euler_from_quaternion(r):
    tmp = (r.x, r.y, r.z, r.w)
    return tf.transformations.euler_from_quaternion(tmp)


def constrain_value(amt, low, high):

    if (math.isnan(amt)):
        return (low + high) * 0.5
    if (amt < low):
        return low

    if (amt > high):
        return high

    return amt


def is_close(x, y):

    return abs(x - y) < 1E-6

if __name__ == '__main__':

    TEST = True
    s3 = 3**0.5

    test_cases = [
        (
            constrain_value,
            [
                ([float('nan'), 0, 1], 0.5),
                ([2, 0, 1], 1),
                ([2, 3, 4], 3),
            ]
        ),
        (
            get_yaw_from_direction,
            [
                ([[0, 0, 0]], math.pi * 0 / 6.0),
                ([[s3, 1, 0]], math.pi * 1 / 6.0),
                ([[1, s3, 0]], math.pi * 2 / 6.0),
                ([[0, 1, 0]], math.pi * 3 / 6.0),
                ([[-1, s3, 0]], math.pi * 4 / 6.0),
                ([[-s3, 1, 0]], math.pi * 5 / 6.0),
                ([[-1, 0, 0]], math.pi * 6 / 6.0),
            ]
        ),
    ]

    if TEST:
        for func, test_case in test_cases:
            print ">>", func.func_name
            for input, result in test_case:
                try:
                    assert is_close(func(*input), result)
                    print "   P :",
                except AssertionError:
                    print "   X :", "Expect", result,
                print input, ">>", result
