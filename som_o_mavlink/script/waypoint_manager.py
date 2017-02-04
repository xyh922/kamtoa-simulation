#!/usr/bin/env python

from pymavlink.dialects.v10 import common as mavlink

from gcs_tele import GcsTelemetry

from waypoint import Waypoint as WP


import rospy


class GcsWaypointManager(GcsTelemetry):
    def __init__(self):

        rospy.loginfo("init GcsWaypointManager !!")

        try:
            rospy.init_node("gcs_waypoint")
        except:
            pass

        self.wp_target_component = None
        self.wp_target_system = None

        self.wp_recv_seq = 0

        self.wp_manager_state = None

        self.wp_waypoints = []

        #self.mission = GcsMissionController(self)

        self.home = WP(32.703177, -117.250314, 113.789)

        print "----"
        print "Home location"
        print self.home

        super(GcsWaypointManager, self).__init__()

    def mission_manager(self, mavmsg):

        mavdict = mavmsg.to_dict()

        self.set_mav_target_from_mavdict(mavdict)

        mav_type = mavdict["mavpackettype"]

        if mav_type == "MISSION_COUNT":  # receive count of waypoint from  QGC
            self.wp_recv_seq = 0
            self.wp_waypoints = []
            self.wp_count = int(mavdict["count"])

            self.mission_request_send(self.wp_recv_seq)

        elif mav_type == "MISSION_ITEM":  # receive waypoint from  QGC
            self.wp_waypoints.append(WP(mavdict=mavdict))

            if self.wp_recv_seq == self.wp_count:  # already got all waypoints
                self.mission_ack_send()

                # start mission
                self.mission_waypoint_set(self.wp_waypoints)
                self.mission_start(waypoints=self.wp_waypoints)

            elif self.wp_recv_seq < self.wp_count:
                self.mission_request_send(self.wp_recv_seq)  # wp_recv_seq

    def set_mav_target_from_mavdict(self, mavdict):
        self.wp_target_system = mavdict['target_system']
        self.wp_target_component = mavdict['target_component']

    def mission_ack_send(self, type=mavlink.MAV_MISSION_ACCEPTED):
        self.mav.mission_ack_send(
            self.wp_target_system, self.wp_target_component, type)

    def mission_request_send(self, seq):
        self.mav.mission_request_send(
            self.wp_target_system, self.wp_target_component, seq)
        self.wp_recv_seq += 1


if __name__ == '__main__':
    x = GcsWaypointManager()
    rospy.spin()
