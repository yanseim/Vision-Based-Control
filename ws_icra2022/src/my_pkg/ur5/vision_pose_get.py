#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
# from ar_track_alvar_msgs.msg import AlvarMarkers
#from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from frompitoangle import *
import os, time
import sys

class VisionPosition():
    def __init__(self, name = "hololens_info_subscriber"):
        self.name = name
        self.pos = []
        self.k_pos = 0

    def Init_node(self):
        rospy.init_node(self.name)
        sub = rospy.Subscriber("/aruco_single/pixel", PointStamped, self.callback)
        return sub

    def callback(self, msg):
        print([msg.point.x,msg.point.y])
        self.pos = list([msg.point.x,msg.point.y])
        self.k_pos=msg.header.seq

if __name__ == "__main__":
    vision_info_reader = VisionPosition()
    vision_info_reader.Init_node()
    while not rospy.is_shutdown():
        # print(vision_info_reader.pos)
        pass
