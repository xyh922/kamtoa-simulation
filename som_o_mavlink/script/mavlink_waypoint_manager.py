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
from waypoint import Waypoint as WP
##############################################################################
# Class
##############################################################################

class MavlinkWaypointManager(object):
    '''
    Waypoint Manager to manage Mission Command
    - Home point
    - Waypoint list [ Mission Manager ]
    '''
    def __init__(self, SingletonMavlinkInterface):
        # Singleton MAV to use send function
        self.mav = SingletonMavlinkInterface.mav

        # Subscribe to mission message
        rospy.Subscriber('/mavlink/mission', MavlinkMsg, self.mission_callback)
        rospy.loginfo("[MAV] Waypoint Manager Initialized")
        
        # Received waypoint detail
        self.wp_target_component = None
        self.wp_target_system = None
        self.wp_recv_seq = 0
        self.wp_count = 0
        self.wp_manager_state = None
        self.wp_waypoints = []

        # Default home Localtion
        self.home = WP(13.736756, 100.533621, 113.789)
        # TODO: Service to change home location

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
            self.wp_recv_seq = 0
            self.wp_waypoints = []
            self.wp_count = int(mavdict["count"])
            # Request more mission
            self.mission_request_send(self.wp_recv_seq)

        # While receiving Waypoints from GCS
        elif mav_type == "MISSION_ITEM":
            self.wp_waypoints.append(WP(mavdict=mavdict))

            # Complete receiving mission
            if self.wp_recv_seq == self.wp_count:
                self.mission_ack_send()
                # TODO : Start mission Routine goes here
                self.show_mission()
                # self.mission_waypoint_set(self.wp_waypoints)
                # self.mission_start(waypoints=self.wp_waypoints)

            elif self.wp_recv_seq < self.wp_count:
                self.mission_request_send(self.wp_recv_seq)  # wp_recv_seq

    def show_mission(self):
        '''
        (Debug) Print mission(waypoint) list
        '''
        print "Mission Received ========"
        for x in self.wp_waypoints:
            print str(x.lat) + "," + str(x.lng) + "," + str(x.alt)
            print "================="
        print "================"

    def set_mav_target_from_mavdict(self, mavdict):
        '''
        Set target of GCS for Robot to contact
        '''
        self.wp_target_system = mavdict['target_system']
        self.wp_target_component = mavdict['target_component']

    def mission_ack_send(self, type=mavlink.MAV_MISSION_ACCEPTED):
        '''
        Acknowledge mission receiving
        '''
        self.mav.mission_ack_send(
            self.wp_target_system, self.wp_target_component, type)

    def mission_request_send(self, seq):
        '''
        Request mission message
        Precondition : Mission count received
        '''
        self.mav.mission_request_send(
            self.wp_target_system, self.wp_target_component, seq)
        self.wp_recv_seq += 1

