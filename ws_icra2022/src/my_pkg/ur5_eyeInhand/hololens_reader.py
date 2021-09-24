#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from frompitoangle import *
import os, time
import sys

class HololensPosition():
    def __init__(self, name = "hololens_info_subscriber"):
        self.name = name
        self.pos = []
        self.k_pos = 0
        print("class is intialized!")
        

    def Init_node(self):
        rospy.init_node(self.name)
        
        sub = rospy.Subscriber("UnityJointStatePublish", JointState, self.callback)
        print('Initialization done!')
        return sub

    def callback(self, msg):
        print(msg.position)
        self.pos = msg.position
        self.k_pos=msg.header.seq

if __name__ == "__main__":
    hololens_info_reader = HololensPosition()   
    hololens_info_reader.Init_node()
    while not rospy.is_shutdown():
        # print(vision_info_reader.pos)
        pass
