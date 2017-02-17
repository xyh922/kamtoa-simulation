#!/usr/bin/env python
"""
Mavlink Waypoint Manager
- Manage Point location of Robot including Home posision
"""
# Author : universez , c3mx
# Date : 6-Feb-2017
##############################################################################
# Imports
##############################################################################
import rospy
from pymavlink.dialects.v10 import common as mavlink
from mavros_msgs.msg import Mavlink as MavlinkMsg
from mavros import mavlink as mavros
from custom_util.waypoint import Waypoint as WP
##############################################################################
# Class
##############################################################################

class MavlinkMissionManager(object):
    '''
    Mission Manager to manage Mission Command
    - Home point
    - Waypoint list [ Mission Manager ]
    '''
    def __init__(self, SingletonMavlinkInterface, WaypointManager):
        # Singleton MAV to use send function
        self.mav = SingletonMavlinkInterface.mav
        self.waypoints = WaypointManager

        # Subscribe to mission message
        rospy.Subscriber('/mavlink/mission', MavlinkMsg, self.mission_callback)
        rospy.loginfo("[MAV] Waypoint Manager Initialized !")

        # Received waypoint detail
        self.waypoints.wp_target_component = None
        self.waypoints.wp_target_system = None
        self.waypoints.wp_recv_seq = 0
        self.waypoints.wp_count = 0
        self.waypoints.wp_manager_state = None
        self.waypoints.wp_waypoints = []

    def mission_callback(self, data, debug=False):
        '''
        Receive Mission
        '''
        buff = mavros.convert_to_bytes(data)
        mavmsg = self.mav.decode(buff)

        mavdict = mavmsg.to_dict()
        self.set_mav_target_from_mavdict(mavdict)
        mav_type = mavdict["mavpackettype"]

        # Two type of mission command
        # Waypoint list init message - "MISSION_COUNT"
        if mav_type == "MISSION_COUNT":
            self.waypoints.wp_recv_seq = 0
            self.waypoints.wp_waypoints = []
            self.waypoints.wp_count = int(mavdict["count"])
            # Request more mission
            self.mission_request_send(self.waypoints.wp_recv_seq)

        # While receiving Waypoints from GCS
        elif mav_type == "MISSION_ITEM":
            self.waypoints.wp_waypoints.append(WP(mavdict=mavdict))

            # Complete receiving mission
            if self.waypoints.wp_recv_seq == self.waypoints.wp_count:
                self.mission_ack_send()
                print "Finish received mission of : " + str(len(self.waypoints.wp_waypoints)) + "points"
                self.show_mission()

            elif self.waypoints.wp_recv_seq < self.waypoints.wp_count:
                self.mission_request_send(self.waypoints.wp_recv_seq)  # wp_recv_seq

    def show_mission(self):
        '''
        (Debug) Print mission(waypoint) list
        '''
        print "Mission Received ========"
        for point in self.waypoints.wp_waypoints:
            print str(point.lat) + "," + str(point.lng) + "," + str(point.alt)
            print "================="
        print "================"

    def set_mav_target_from_mavdict(self, mavdict):
        '''
        Set target of GCS for Robot to contact
        '''
        self.waypoints.wp_target_system = mavdict['target_system']
        self.waypoints.wp_target_component = mavdict['target_component']

    def mission_ack_send(self, statustype=mavlink.MAV_MISSION_ACCEPTED):
        '''
        Acknowledge mission receiving
        '''
        self.mav.mission_ack_send(
            self.waypoints.wp_target_system, self.waypoints.wp_target_component, statustype)

    def mission_request_send(self, seq):
        '''
        Request mission message
        Precondition : Mission count received
        '''
        self.mav.mission_request_send(
            self.waypoints.wp_target_system, self.waypoints.wp_target_component, seq)
        self.waypoints.wp_recv_seq += 1

