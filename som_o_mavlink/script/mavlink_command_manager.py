#!/usr/bin/env python
"""
Mavlink Command Message Manager Module
"""
# Author : universez , c3mx
# Date : 6-Feb-2017
##############################################################################
# Imports
##############################################################################
import rospy
from mavlink_robot_odom import MavlinkRobotOdometry
from mavlink_heartbeat import MavlinkHeartbeatGenerator
from pymavlink.dialects.v10 import common as mavlink
from mavros_msgs.msg import Mavlink as MavlinkMsg
from mavros import mavlink as mavros

##############################################################################
# Class
##############################################################################

class MavlinkCommandManager(object):
    '''
    Manager for COMMAND_ Message
    '''
    def __init__(self, SingletonMavlinkInterface, StatusManager, WaypointContainer, Executor):
        rospy.loginfo("[MAV] Command manager Initialized !")
        self.mav = SingletonMavlinkInterface.mav
        self.status_manager = StatusManager
        # Robot Odometry to send odometry periodically
        self.robot_odom = MavlinkRobotOdometry(SingletonMavlinkInterface,
                                               WaypointContainer, StatusManager)
        # Robot Heartbeat (Set Status Flag)
        self.robot_heart = MavlinkHeartbeatGenerator(SingletonMavlinkInterface)
        # Robot Mission Executor class pointer
        self.robot_executor = Executor
        # Waypoints
        self.robot_waypoints = WaypointContainer
        # Subscribe to command received
        rospy.Subscriber('/mavlink/command', MavlinkMsg, self.command_callback)

    def command_callback(self, data):
        '''
        Receive command
        '''
        buff = mavros.convert_to_bytes(data)
        mavmsg = self.mav.decode(buff)
        mavmsg_type = mavmsg.get_type()
        if mavmsg_type == "COMMAND_LONG":
            # Initial Command
            if mavmsg.command == mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                self.robot_odom.send_init_information()
            #ARM/DISARM
            if mavmsg.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if mavmsg.param1 == 1:
                    print "trying to arm"
                    self.mav.command_ack_send(mavmsg.command, mavlink.MAV_RESULT_ACCEPTED)
                    self.robot_heart.base_mode = mavlink.MAV_MODE_AUTO_ARMED
                    self.robot_executor.mission_copy_from()
                    self.robot_executor.mission_start()
                elif mavmsg.param1 == 0:
                    print "trying to disarm"
                    self.mav.command_ack_send(mavmsg.command, mavlink.MAV_RESULT_ACCEPTED)
                    # Reset Everything
                    self.robot_executor.mission_repeat_set(False)
                    self.robot_heart.base_mode = mavlink.MAV_MODE_AUTO_DISARMED
                    self.robot_executor.mission_stop()
            #GETCURRENTWAYPOINT
            #GOT DO JUMP
            if mavmsg.command == mavlink.MAV_CMD_DO_JUMP:
                if mavmsg.param2 == -1:
                    print "Do Jump Activated"
                    # Do jump forever
                    #jump_to_waypoint_no = mavmsg.param1
                    self.robot_executor.mission_repeat_set(True)
                if mavmsg.param1 == -1:
                    print "Do Jump Deactivated"
                    self.robot_executor.mission_repeat_set(False)
            if mavmsg.command == mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                # Go to home position (DO RETURN TO LAUNCH)
                self.robot_executor.go_to_origin()
        if mavmsg_type =="SET_MODE" and mavmsg.custom_mode == 6:
            # RETURN TO LAUNCH FLAG
            self.robot_executor.go_to_origin()
