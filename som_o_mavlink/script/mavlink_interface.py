#!/usr/bin/env python
"""Receive Original Mavlink messages
    and transform into ROS-Mavlink messages"""
# Mavlink Interface
# Author : universez , c3mx
# Date : 3-Feb-2017
##############################################################################
# Imports
##############################################################################
from mavros import mavlink as mavros
import rospy
from mavros_msgs.msg import Mavlink as MavlinkMsg
from pymavlink.dialects.v10 import common as mavlink

from mavlink_heartbeat import MavlinkHeartbeatGenerator
from mavlink_telemetry import MavlinkTelemetry
from mavlink_waypoint_manager import MavlinkWaypointManager
##############################################################################
# Class
##############################################################################

class MavlinkCommunication(object):
    '''
    Interface class for communicating with MAV message from GCS_BRIDGE
    Input : ROS-MAV-MSG from GCS_BRIDGE
    '''
    def __init__(self):
        rospy.loginfo("[interface] init MavLink Main Communication !!")
        try:
            rospy.init_node("gcs_mavlink")
        except:
            pass

        # Publisher MAV messages from GCS to ROS
        self.pub_mavlink = rospy.Publisher(
            '/mavlink/to_gcs', MavlinkMsg, queue_size=1)
        # Subscribe ROS Message , packed and sent to GCS via MAV
        rospy.Subscriber('/mavlink/from_gcs', MavlinkMsg, self.mav_callback)
        # Publisher MAV Mission to Mission Manager
        self.pub_mission = rospy.Publisher(
            '/mavlink/mission', MavlinkMsg, queue_size=1)
        # Publisher MAV Command to Command Manager
        self.pub_command = rospy.Publisher(
            '/mavlink/command', MavlinkMsg, queue_size=1)
        # dummy file
        dummyfile = {}
        # Link to MAV Software
        self.mav = mavlink.MAVLink(dummyfile, srcSystem=1, srcComponent=10)
        # override send function
        self.mav.send = self.mavsend
        # Params Set
        self.debug_read = True
        self.debug_blacklist = []


    def mav_callback(self, data, debug=False):
        '''Callback from ROS Side receive message from GCS'''
        buff = mavros.convert_to_bytes(data)
        mavmsg = self.mav.decode(buff)
        mavmsg_type = mavmsg.get_type()

        # Send MAV message to each corresponding converter type
        if mavmsg_type.startswith("MISSION_"):
            # Require inheritance to gcs_waypoint class
            print "mission_ received"
            self.pub_mission.publish(data)#self.mission_manager(mavmsg)

        elif mavmsg_type.startswith("COMMAND_"):
            # Require inheritance to gcs_bridge class
            print "command_ received"
            self.pub_command.publish(data)#self.command_manager(mavmsg)

        if mavmsg_type not in self.debug_blacklist:
            print "<<RECEIVING<<"
            print mavmsg
            print

    def mavsend(self, mavmsg, force_mavlink1=False):
        '''
        (Override send function of mavros)
        Send ROS Command to GCS
        '''
        #print mavmsg
        buff = mavmsg.pack(self.mav)
        self.mav.seq = (self.mav.seq + 1) % 256
        rosmsg = mavros.convert_to_rosmsg(mavmsg)
        self.pub_mavlink.publish(rosmsg)


if __name__ == '__main__':
    mi = MavlinkCommunication()
    # Set Home Position, Location calculation, Waypoint Manager
    waypoint = MavlinkWaypointManager(mi)
    heartbeat = MavlinkHeartbeatGenerator(mi)
    # Data from Robot (require Waypoint Manager)
    telemetry = MavlinkTelemetry(mi, waypoint)

    rospy.spin()
