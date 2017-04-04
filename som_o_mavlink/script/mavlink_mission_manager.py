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
import json
import os
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
        self.wp_file = None
        self.waypoints.wp_target_component = None
        self.waypoints.wp_target_system = None
        self.waypoints.wp_recv_seq = 0
        self.waypoints.wp_count = 0
        self.waypoints.wp_manager_state = None
        self.waypoints.wp_waypoints = []

        # Load Last Mission
        self.read_from_file(self.resolve_filename("waypoints.wp"))

    def resolve_filename(self, relative_filename):
        '''
        Resolve filename to the relative path to the script
        '''
        script_dir = os.path.dirname(__file__)
        rel_path = relative_filename
        abs_file_path = os.path.join(script_dir, rel_path)
        return abs_file_path

    def write_to_file(self, filename):
        '''
        Write current holding waypoints to a file
        '''
        if len(self.waypoints.wp_waypoints) == 0:
            return
        # Normal Condition
        self.wp_file = open(filename, 'w')
        json_string = json.dumps([wp.to_dict() for wp in self.waypoints.wp_waypoints])
        self.wp_file.write(json_string)
        self.wp_file.close()

    def read_from_file(self, filename):
        '''
        Read Waypoints from a file
        '''
        print "[MAV] Loading waypoints from file" + filename
        with open(filename, 'r+') as self.wp_file:
            try:
                print "[MAV] Loading"
                readFile = json.loads(str(self.wp_file.read())) # Got Dict json key:value
                self.waypoints.wp_waypoints = map(lambda point : WP(selfdict = point), readFile)
                # for point in readFile:
                    # self.waypoints.wp_waypoints.append(WP(selfdict=point))
                self.waypoints.wp_count = len(self.waypoints.wp_waypoints)
                print "[MAV] Length : " + str(len(self.waypoints.wp_waypoints))
            except ValueError:
                print "[MAV] Error ValueError"
                self.waypoints.wp_count = 0
                self.waypoints.wp_waypoints = []
        self.wp_file.close()

    def mission_callback(self, data):
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
                print "Finish received mission of : " + str(len(self.waypoints.wp_waypoints))
                self.write_to_file(self.resolve_filename("waypoints.wp"))
                self.show_mission()
            elif self.waypoints.wp_recv_seq < self.waypoints.wp_count:
                self.mission_request_send(self.waypoints.wp_recv_seq)

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
