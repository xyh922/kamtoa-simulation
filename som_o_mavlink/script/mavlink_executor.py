#!/usr/bin/env python
"""
Waypoint executor
- Send command to move base sequentially
"""
# Mavlink Interface
# Author : universez , c3mx
# Date : 10-Feb-2017
##############################################################################
# Imports
##############################################################################
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID

from custom_util.utils import get_yaw_from_direction, to_list, quaternion_from_euler
##############################################################################
# Class
##############################################################################
class MavlinkExecutor(object):
    '''
    docstring for GcsWaypointController
    '''
    def __init__(self, SingletonMavlinkInterface, WaypointManager, StatusManager):
        rospy.loginfo("[MAV] Mission Executor Initilized !")
        # Singleton MAV to use send function
        self.mav = SingletonMavlinkInterface.mav
        self.waypoints = WaypointManager
        self.mission_current_wp = 0 # Waypoint idx
        self.mission_waypoints = [] # list for waypoints
        self.mission_repeat = False
        self.status_manager = StatusManager
        self.return_to_home_call = False

        # Destination topics
        self.pub_goal = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=1)

        self.pub_goal_cal = rospy.Publisher(
            '/move_base/cancel', GoalID, queue_size=1)

        rospy.Subscriber('/move_base/result',
                         MoveBaseActionResult, self.mission_wp_reached)

    def go_to_origin(self):
        '''
        Go back to origin position (assuming HOME)
        '''
        # Cancle current mission
        self.mission_stop()
        self.return_to_home_call = True
        # Set the origin point
        goal = self.get_ros_pose_msg(self.waypoints.home)
         # Reset Stats
        self.mission_current_wp = 0
        # Issue the goal to move_base action server 
        rospy.loginfo("[MAV] Return to Launch")
        self.pub_goal.publish(goal)
       
    def mission_repeat_set(self, flag):
        '''
        Set Repeat flag
        '''
        self.mission_repeat = flag

    def mission_wp_has_next(self):
        '''
        After End single waypoint , here is the code to iterate through
        '''
        if not self.mission_repeat:
            rospy.loginfo("[MAV] Normal Iterate to next waypoint")
            self.mission_current_wp += 1
            if self.mission_current_wp < len(self.mission_waypoints):
                self.mission_wp_start(seq=self.mission_current_wp)
                return True
        else:
            # Calculate Next position
            self.mission_current_wp += 1
            self.mission_current_wp = self.mission_current_wp % len(self.mission_waypoints)
            # Send next mission
            self.mission_wp_start(seq=self.mission_current_wp)
            rospy.loginfo("[MAV] Looping with Next Goal : " + str(self.mission_current_wp))
            return True
        return False

    def mission_wp_reached(self, msg):
        '''
        Task to do when reach a single waypoint
        - Check the next WP
        - Decide whether to go or not (Receive Mission)
        '''
        rospy.loginfo("[MAV] Waypoint reached status ENUM : " + str(msg.status.status))
        if msg.status.status == 2:  # Canceled
            return

        if self.return_to_home_call:
            self.return_to_home_call = False
            return

        self.mav.mission_item_reached_send(self.mission_current_wp)
        has_next = self.mission_wp_has_next()  # Do next mission
        if not has_next:
            rospy.loginfo("[MAV] Mission finish !!")

    def mission_copy_from(self):
        '''
        Copy the mission from the receiving buffer
        '''
        print "Copy from " + str(len(self.waypoints.wp_waypoints))
        self.mission_waypoints = self.waypoints.wp_waypoints

    def mission_wp_start(self, seq):
        '''
        Send the goal to Navigation Stack Server
        '''
        wp = self.mission_waypoints[seq]
        goal = self.get_ros_pose_msg(wp)
        rospy.loginfo("[MAV] Going to goal : "+str(seq)+" : " + str(wp.lat) + "," + str(wp.lng))

        self.pub_goal.publish(goal)
        self.mav.mission_current_send(self.mission_current_wp)

    def mission_stop(self):
        '''
        Cancel Current goal
        '''
        self.pub_goal_cal.publish(GoalID())

    def mission_start(self, waypoints=None):
        '''
        Start the mission (from the waypoint[index] = 0)
        '''
        rospy.loginfo("[MAV] ARMED - Mission start with waypoint counts : " + str(len(self.mission_waypoints)))
        if waypoints:
            self.mission_waypoint_set(waypoints)

        self.pub_goal_cal.publish(GoalID())
        self.mission_current_wp = 0
        self.mission_wp_start(seq=self.mission_current_wp)

    def mission_waypoint_set(self, waypoints):
        '''
        Set Waypoint (Call from outside)
        '''
        self.mission_waypoints = waypoints

    def get_ros_pose_msg(self, wp):
        '''
        Pack pose message to be sent to Navigation Stack server
        '''
        x = wp.lat
        y = wp.lng
        z = 0

        rosmsg = PoseStamped()

        rosmsg.header.stamp = rospy.Time.now()
        rosmsg.header.frame_id = "odom"

        rosmsg.pose.position.x = x
        rosmsg.pose.position.y = y  # revert axis
        rosmsg.pose.position.z = 0
        # print wp

        direction = map(lambda x, y: x - y, to_list(rosmsg.pose.position),
                        to_list(self.status_manager.state.pose.pose.position))

        yaw = get_yaw_from_direction(direction)
        rosmsg.pose.orientation = quaternion_from_euler(0, 0, yaw)
        return rosmsg

