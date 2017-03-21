#!/usr/bin/env python
"""
Report Heartbeat to GCS
according to Mavlink Protocol
"""
# Mavlink Interface
# Author : universez , c3mx
# Date : 6-Feb-2017
##############################################################################
# Imports
##############################################################################
import rospy
from pymavlink.dialects.v10 import common as mavlink
##############################################################################
# Class
##############################################################################

class MavlinkHeartbeatGenerator(object):
    '''
    Report current status and health of the unit
    => Heartbeat report
    '''
    def __init__(self, SingletonMavlinkInterface):
        self.mav = SingletonMavlinkInterface.mav
        # Default state
        self.state = mavlink.MAV_STATE_ACTIVE
        # Default MODE
        #self.base_mode = mavlink.MAV_MODE_AUTO_DISARMED #armed
        self.base_mode = mavlink.MAV_MODE_GUIDED_DISARMED
        # self.base_mode |= mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
        # self.base_mode |= mavlink.MAV_MODE_FLAG_AUTO_ENABLED

        rospy.loginfo("[MAV] Heartbeat Generator Initiated")
        rospy.Timer(rospy.Duration(1.0), self.heartbeat_send)

    def set_state(self, state):
        '''
        Set the state of the heartbeat status
        '''
        self.state = state

    def heartbeat_send(self, event):
        '''
        Create a Mavlink Heartbeat message
        and send to the GCS
        '''
        vehicle_type = mavlink.MAV_TYPE_GROUND_ROVER
        autopilot_option = mavlink.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY

        base_mode = self.base_mode

        custom_mode = 0
        system_status = self.state
        mavlink_version = 3

        # Send message through mav
        self.mav.heartbeat_send(
            vehicle_type,
            autopilot_option,
            base_mode,
            custom_mode,
            system_status,
            mavlink_version)
