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
from nav_msgs.msg import Odometry
import geometry_msgs.msg

from mavlink_mission_manager import MavlinkMissionManager
from mavlink_waypoint import MavlinkWaypoint
from mavlink_executor import MavlinkExecutor
from mavlink_command_manager import MavlinkCommandManager
##############################################################################
# Class
##############################################################################

class MavlinkCommunication(object):
    '''
    Interface class for communicating with MAV message from GCS_BRIDGE
    Input : ROS-MAV-MSG from GCS_BRIDGE
    '''
    def __init__(self):
        rospy.loginfo("[MAV] Mavlink Communication Interface Ready !")
        try:
            rospy.init_node("gcs_mavlink")
        except:
            pass
        # get the component id and system id of this client
        self.component_id = rospy.get_param('~component_id', "10")
        self.system_id = rospy.get_param('~system_id', "1")

        # Publisher MAV messages to GCS
        self.pub_mavlink = rospy.Publisher(
            '/mavlink/to_gcs', MavlinkMsg, queue_size=10)
        # Subscribe ROS Message which received from GCS
        rospy.Subscriber('/mavlink/from_gcs', MavlinkMsg, self.mav_callback)
        # Publisher MAV Mission to Mission Manager
        self.pub_mission = rospy.Publisher(
            '/mavlink/mission', MavlinkMsg, queue_size=1)
        # Publisher MAV Command to Command Manager
        self.pub_command = rospy.Publisher(
            '/mavlink/command', MavlinkMsg, queue_size=1)
        # Publisher for Manual controlling [Teleop]
        self.pub_cmd_vel = rospy.Publisher(
            '/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        # dummy file
        dummyfile = {}
        # Create link to MAV Software
        self.mav = mavlink.MAVLink(dummyfile, srcSystem= self.system_id , srcComponent=self.component_id)
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
        mavdict = mavmsg.to_dict()
        # Command and mission Message Receiving
        if 'target_system' in mavdict.keys():
            system_id_ = mavdict['target_system']
            # Match ID
            if self.system_id == system_id_:
                # Send MAV message to each corresponding converter type
                if mavmsg_type.startswith("MISSION_"):
                    # Require inheritance to gcs_waypoint class
                    print "mission_ received"
                    self.pub_mission.publish(data)

                elif mavmsg_type.startswith("COMMAND_"):
                    # Require inheritance to gcs_bridge class
                    print "command_ received"
                    print "command=== : " + str(mavmsg_type)
                    self.pub_command.publish(data)
                
        # Manual control Message receiving
        if 'target' in mavdict.keys():
            system_id_ = mavdict['target']
            # Match ID
            if self.system_id == system_id_:
                if mavmsg_type.startswith("MANUAL_CONTROL"):
                    # Reference common.py 3922 MANUAL_CONTROL
                    cmd = geometry_msgs.msg.Twist()
                    cmd.linear.x = float(mavdict['x'])/1000
                    cmd.angular.z = float(mavdict['r'])/1000
                    self.pub_cmd_vel.publish(cmd)

        # Debugging Print
        if mavmsg_type not in self.debug_blacklist:
            print "<<RECEIVING FROM GCS<<"
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

class MavlinkStatusManager(object):
    def __init__(self):
        self.state = Odometry()

if __name__ == '__main__':
    status = MavlinkStatusManager()
    # Main MAV Communication handler
    mi = MavlinkCommunication()
    # Centralized Waypoint Data
    waypoint = MavlinkWaypoint()
    # Mission Executor - Waypoint follower
    executor = MavlinkExecutor(mi, waypoint, status)
    # Set Home Position, Location calculation, Waypoint Manager
    mission_manager = MavlinkMissionManager(mi, waypoint)
    # Command receive and sending 
    command_manager = MavlinkCommandManager(mi, status, waypoint, executor)
    # Teleoperation Daemon 
    teleop = "mission_manager,command_manager"

    rospy.spin()
