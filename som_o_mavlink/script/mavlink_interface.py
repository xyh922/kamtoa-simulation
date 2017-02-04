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

##############################################################################
# Class
##############################################################################

class GcsMavlink(object):
    '''
    Interface class for communicating with MAV message from GCS_BRIDGE
    Input : ROS-MAV-MSG from GCS_BRIDGE
    '''
    def __init__(self):
        rospy.loginfo("[interface] init GcsMavlink !!")
        try:
            rospy.init_node("gcs_mavlink")
        except Exception:
            pass

        # Publisher MAV messages from GCS to ROS
        self.pub_mavlink = rospy.Publisher(
            '/mavlink/from', MavlinkMsg, queue_size=1)
        # Subscribe ROS Message , packed and sent to GCS via MAV
        rospy.Subscriber('/mavlink/to', MavlinkMsg, self.mav_callback)

        # dummy file
        dummyfile = {}
        # Link to MAV Software
        self.mav = mavlink.MAVLink(dummyfile, srcSystem=1, srcComponent=10)
        # Map function
        self.mav.send = self.mavsend
        # Params Set
        self.debug_read = True
        self.debug_blacklist = []

    def mav_callback(self, data, debug=False):
        pass
    #     '''Callback from ROS Side to send message to GCS'''
    #     buff = mavros.convert_to_bytes(data)
    #     mavmsg = self.mav.decode(buff)
    #     mavmsg_type = mavmsg.get_type()

    #     # Send MAV message to each corresponding converter type
    #     if mavmsg_type.startswith("MISSION_"):
    #         # Require inheritance to gcs_waypoint class
    #         print "mission_ received"
    #         #self.mission_manager(mavmsg)
    #     elif mavmsg_type.startswith("COMMAND_"):
    #         # Require inheritance to gcs_bridge class
    #         #self.command_manager(mavmsg)
    #         print "command_ received"

    #     if mavmsg_type not in self.debug_blacklist:
    #         print "<<<"
    #         print mavmsg
    #         print

    def mavsend(self, mavmsg, force_mavlink1=False):
        '''
        (Override send function of mavros)
        Send GCS Command to ROS
        '''

        print mavmsg
        buff = mavmsg.pack(self.mav)
        self.mav.seq = (self.mav.seq + 1) % 256

        rosmsg = mavros.convert_to_rosmsg(mavmsg)

        # mavmsg_type = mavmsg.get_type()

        # if mavmsg_type not in self.debug_blacklist:
        #     print ">>>"
        #     print mavmsg
        #     print
        self.pub_mavlink.publish(rosmsg)

    def heartbeat_send(self):
        type = 2#mavlink.MAV_TYPE_GROUND_ROVER#MAV_TYPE_SUBMARINE
        autopilot = mavlink.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY

        base_mode = mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        base_mode |= mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
        base_mode |= mavlink.MAV_MODE_FLAG_AUTO_ENABLED

        custom_mode = 0
        system_status = mavlink.MAV_STATE_ACTIVE
        mavlink_version = 3

        self.mav.heartbeat_send(
            type, autopilot, base_mode, custom_mode, system_status, mavlink_version)


if __name__ == '__main__':
    x = GcsMavlink()

    rate = rospy.Rate(10) # 10Hz

    while not rospy.is_shutdown():
        
        x.heartbeat_send()

        rate.sleep()
    rospy.spin()