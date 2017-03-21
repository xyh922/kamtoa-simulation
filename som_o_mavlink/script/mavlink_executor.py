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
from std_msgs.msg import Float64

from custom_util.utils import get_yaw_from_direction, to_list, quaternion_from_euler
from math import degrees

##############################################################################
# Class
##############################################################################


class MavlinkExecutor(object):
    """docstring for GcsWaypointController"""

    def __init__(self, SingletonMavlinkInterface, WaypointManager,StatusManager):
        rospy.loginfo("[MAV] Mission Executor Initilized !")
        # Singleton MAV to use send function
        self.mav = SingletonMavlinkInterface.mav
        self.waypoints = WaypointManager
        self.mission_current_wp = 0 #Waypoint idx
        self.mission_waypoints = [] #list for waypoints

        self.status_manager = StatusManager

        # Destination topics
        self.pub_goal = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=1)

        self.pub_goal_cal = rospy.Publisher(
            '/move_base/cancel', GoalID, queue_size=1)

        rospy.Subscriber('/move_base/result',
                         MoveBaseActionResult, self.mission_wp_reached)

    # do next mission
    def mission_wp_next(self):

        self.mission_current_wp += 1

        if self.mission_current_wp < len(self.mission_waypoints):
            self.mission_wp_start(seq=self.mission_current_wp)
            return True

        return False

    def mission_wp_reached(self, msg):

        rospy.loginfo("[MAV] Waypoint reached status ENUM : " + str(msg.status.status))
        if msg.status.status == 2:  # canceled
            return

        self.mav.mission_item_reached_send(self.mission_current_wp)
        has_next = self.mission_wp_next()  # do next mission
        if not has_next:
            rospy.loginfo("[MAV] Mission finish !!")

    def mission_copy_from(self):
        print "Copy from " + str(len(self.waypoints.wp_waypoints))
        self.mission_waypoints = self.waypoints.wp_waypoints

    def mission_wp_start(self, seq):

        wp = self.mission_waypoints[seq]
        goal = self.get_ros_pose_msg(wp)

        rospy.loginfo("[MAV] Going to goal : "+str(seq)+" : " + str(wp.lat) + "," + str(wp.lng)); 

        self.pub_goal.publish(goal)
        self.mav.mission_current_send(self.mission_current_wp)

    def mission_stop(self):
        self.pub_goal_cal.publish(GoalID())


    def mission_start(self, waypoints=None):
        rospy.loginfo("[MAV] ARMED - Mission start with waypoint counts : " + str(len(self.mission_waypoints)))
        if waypoints:
            self.mission_waypoint_set(waypoints)

        self.pub_goal_cal.publish(GoalID())
        self.mission_current_wp = 0

        self.mission_wp_start(seq=self.mission_current_wp)

    def mission_waypoint_set(self, waypoints):
        self.mission_waypoints = waypoints

    def get_ros_pose_msg(self, wp):

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

