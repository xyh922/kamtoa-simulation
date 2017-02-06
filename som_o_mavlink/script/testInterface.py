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

##############################################################################
# Class
##############################################################################
class Mav():
    seq = 0
    def send(self, mavmsg, force_mavlink1=False):
        print "fuck this send"

class MavInterface(object) :
    def __init__(self) :
        self.mav = Mav()
        self.mav.send = self.custom_mav_send 
    def custom_mav_send(self, mavmsg, force_mavlink1=False):
        self.mav.seq = (self.mav.seq + 1) % 256
        print mavmsg + str(self.mav.seq) 

class Health(object) : 
    def __init__(self,MI) : 
        self.mav = MI.mav
        self.seq = 0
    def send_msg(self):
        self.seq = self.seq+1
        self.mav.send("Health"+str(self.seq)+"===")

class Tele(object) : 
    def __init__(self,MI) : 
        self.mav = MI.mav
        self.seq = 0
        
    def send_msg(self):
        self.seq = self.seq + 5
        self.mav.send("Tele"+str(self.seq)+"===")

MI = MavInterface()
X = Health(MI)
Y = Tele(MI)
for i in range(0,300) :
    X.send_msg()
    Y.send_msg()