#!/usr/bin/env python
"""
Mavlink Robot Odometry Report
"""
# Mavlink Interface
# Author : universez , c3mx
# Date : 6-Feb-2017
##############################################################################
# Imports
##############################################################################
import rospy
from pymavlink.dialects.v10 import common as mavlink
from mavros_msgs.msg import Mavlink as MavlinkMsg
from mavros import mavlink as mavros
from time import sleep, time
from math import pi, radians, degrees
from custom_util.utils import *
from nav_msgs.msg import Odometry

##############################################################################
# Class
##############################################################################

class MavlinkRobotOdometry(object): 
    '''
    Receive ROS Positioning Data and Send to the GCS 
    '''
    def __init__(self, SingletonMavlinkInterface, WaypointContainer,StatusManager):
        self.mav = SingletonMavlinkInterface.mav
        self.home = WaypointContainer.home
        self.status_manager = StatusManager
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.throttle = self.alt = self.time_boot_ms = 0
        self.odom = self.state = None
        self.airspeed = self.boot_time = self.groundspeed = 0
        self.heading = self.climb = 0
        self.roll = self.pitch = self.yaw = 0
        self.vx = self.vy = self.vz = 0
        self.x = self.y = self.z = 0
        self.rollspeed = self.pitchspeed = self.yawspeed = 0
        self.lat = self.home.lat
        self.lon = self.home.lng
        self.alt = self.home.alt
        self.relative_alt = 0
        # Initial Telemetry
        self.rate = 20#20.0
        self.tele_setup()

    def odometry_callback(self, data):
        '''
        Callback on Odometry receival
        '''
        self.status_manager.state = data
        self.odom = data
        position = self.odom.pose.pose.position
        self.x, self.y, self.z = position.x, - position.y, position.z #
        ang = euler_from_quaternion(data.pose.pose.orientation)
        self.roll, self.pitch, self.yaw = ang
        self.vx, self.vy, self.vz, self.rollspeed, self.pitchspeed, self.yawspeed = ttl(
            data.twist.twist)
        self.yaw = self.yaw
        self.yaw = (4 * pi - self.yaw) % (2 * pi)
        self.groundspeed = self.vx
        self.heading = degrees(self.yaw)
        self.relative_alt = self.z
        self.climb = self.vz


    def tele_setup(self):
        '''
        Do Initial Telemetry Setup
        '''
        rospy.loginfo("Tele Setup")
        self.boot_time = time()

        rospy.logwarn("[MAV - Telemetry] Wait for Robot Odometry generator")
        rospy.wait_for_message('/odom', Odometry)

        rospy.loginfo("[MAV - Telemetry] Robot connected !!")
        rospy.sleep(1)

        self.send_init_information()

        rospy.Timer(rospy.Duration(1.0 / self.rate), self.send_information)


    def send_information(self, event=None):
        '''
        Send information (POSE) to GCS
        '''
        self.time_boot_ms = self.get_boot_time()
        self.airspeed = self.groundspeed

        self.mav.vfr_hud_send(self.airspeed, self.groundspeed,
                              self.heading, self.throttle, self.alt, self.climb)

        self.mav.attitude_send(self.time_boot_ms, self.roll, self.pitch,
                               self.yaw, self.rollspeed, self.pitchspeed, self.yawspeed)

        self.mav.local_position_ned_send(
            self.time_boot_ms, self.x, self.y, self.z, self.vx, self.vy, self.vz)

        self.mav.global_position_int_send(
            self.time_boot_ms,
            self.x*1E7,
            #(self.home.lat + 5*self.home.offset_from_meter_lat(self.x) )*1E7,##self.lat * 1E7,
            self.y*1E7,
            #(self.home.lng + 5*self.home.offset_from_meter_long(self.y) )*1E7,##self.lon * 1E7,
            self.alt * 1000,
            self.relative_alt * 1000,
            self.vx * 100,
            self.vy * 100,
            self.vz * 100,
            self.heading* 100)
        # TO DO :  255, 255, 255, 255, 255

        self.mav.gps_raw_int_send(
            self.time_boot_ms * 1000,
            3,
            self.x*1E7,
            self.y*1E7,
            self.alt * 1000,
            255, 255, 255, 255, 255)

    def send_init_information(self):
        '''
        Routine to send the initial information
        '''
        self.home_position_send()

    def home_position_send(self):
        '''
        Set Home Position
        '''
        self.mav.home_position_send(
            self.home.lat * 1E7,
            self.home.lng * 1E7,
            self.home.alt * 1000,
            x=0, y=0, z=0,
            q=[0] * 4,
            approach_x=0,
            approach_y=0,
            approach_z=0)

    def get_boot_time(self):
        '''
        Get time elasped from last boot time
        '''
        return (time() - self.boot_time) * 1000
