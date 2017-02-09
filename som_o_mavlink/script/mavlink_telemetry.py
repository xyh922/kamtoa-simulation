#!/usr/bin/env python
"""
Mavlink Telemetry Module
- Report Home Position (on initial)
- Report Current Position (subscribing to ROS's odom)
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
from utils import *
from nav_msgs.msg import Odometry

##############################################################################
# Class
##############################################################################


class MavlinkTelemetry(object):
    '''
    Report current status and health of the unit
    => Location report
    => Command Message Manager
    '''

    def __init__(self, SingletonMavlinkInterface, WaypointManager):
        # Singleton MAV to use send function
        self.mav = SingletonMavlinkInterface.mav
        self.home = WaypointManager.home
        self.debug_blacklist = SingletonMavlinkInterface.debug_blacklist
        rospy.loginfo("[MAV] Telemetry & Command manager Initialized")

        # Subscribe to command received
        rospy.Subscriber('/mavlink/command', MavlinkMsg, self.command_callback)

        # Subscribe to states
        # rospy.Subscriber('/states', MavlinkMsg, self.command_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        self.throttle = 0
        self.alt = 0
        self.time_boot_ms = 0

        self.odom = None

        self.airspeed = 0
        self.boot_time = 0
        self.groundspeed = 0
        self.heading = 0
        self.climb = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.rollspeed = 0
        self.pitchspeed = 0
        self.yawspeed = 0
        self.lat = self.home.lat
        self.lon = self.home.lng
        self.alt = self.home.alt
        self.relative_alt = 0
        self.hdg = 0

        self.state = None
        self.tele_rate = 20.0  # 50 Hz
        self.debug_blacklist += [
            "GPS_RAW_INT",
            "GLOBAL_POSITION_INT",
            "LOCAL_POSITION_NED",
            "ATTITUDE",
            "VFR_HUD",
            "HEARTBEAT",
        ]

        self.tele_setup()

    def odometry_callback(self, data, debug=False):
        '''
        Callback on Odometry receival
        '''
        self.odom = data
        position = self.odom.pose.pose.position
        self.x, self.y, self.z = position.x, position.y, position.z
        ang = euler_from_quaternion(data.pose.pose.orientation)
        self.roll, self.pitch, self.yaw = ang
        self.vx, self.vy, self.vz, self.rollspeed, self.pitchspeed, self.yawspeed = ttl(
            data.twist.twist)
        self.yaw = self.yaw  # - radians(90)
        self.yaw = (4 * pi - self.yaw) % (2 * pi)
        self.groundspeed = self.vx
        self.heading = degrees(self.yaw)
        self.hdg = self.heading
        self.relative_alt = self.z
        self.climb = self.vz


    def command_callback(self, data, debug=False):
        '''
        Receive command
        '''
        buff = mavros.convert_to_bytes(data)
        mavmsg = self.mav.decode(buff)
        mavmsg_type = mavmsg.get_type()
        if mavmsg_type == "COMMAND_LONG":
            if mavmsg.command == mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                self.send_init_information()

    def send_init_information(self, event=None):
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

    def tele_setup(self):
        '''
        Do Initial Telemetry Setup
        '''
        rospy.loginfo("Tele Setup")
        self.boot_time = time()

        rospy.logwarn("[MAV - Telemetry] Wait for Robot Connection")

        rospy.wait_for_message('/odom', Odometry)
        #rospy.wait_for_message('/syrena/gps/fix', NavSatFix)

        rospy.loginfo("[MAV - Telemetry] Robot connected !!")
        rospy.sleep(1)

        self.send_init_information()

        rospy.Timer(rospy.Duration(1.0 / self.tele_rate),
                    self.send_information)  # 50 Hz

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
        #print str(self.x) + "," + str(self.y)

        self.mav.global_position_int_send(
            self.time_boot_ms,
            self.x *1E7,#
            self.y *1E7,#
            #self.lat * 1E7,
            #self.lon * 1E7,
            self.alt * 1000,
            self.relative_alt * 1000,
            self.vx * 100,
            self.vy * 100,
            self.vz * 100,
            self.hdg * 100)

        # TO DO :  255, 255, 255, 255, 255
        self.mav.gps_raw_int_send(self.time_boot_ms * 1000, 3, self.lat *
                                  1E7, self.lon * 1E7, self.alt * 1000, 255, 255, 255, 255, 255)
