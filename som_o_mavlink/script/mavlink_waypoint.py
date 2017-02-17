#!/usr/bin/env python
"""
Mavlink Waypoint
- Manage Point location of Robot including Home position
"""
# Author : c3mx
# Date : 10-Feb-2017
##############################################################################
# Imports
##############################################################################
import rospy
from custom_util.waypoint import Waypoint as WP
##############################################################################
# Class
##############################################################################

class MavlinkWaypoint(object):
    '''
    Container for waypoints
    '''
    def __init__(self):
         # Subscribe to mission message
        rospy.loginfo("[MAV] Waypoint Container initialized !")

        # Mission Waypoint detail
        self.wp_target_component = None
        self.wp_target_system = None
        self.wp_recv_seq = 0
        self.wp_count = 0
        self.wp_manager_state = None
        self.wp_waypoints = []

        # Default home Location
        #self.home = WP(13.736756, 100.533621, 113.789)
        self.home = WP(0, 0, 113.789)
