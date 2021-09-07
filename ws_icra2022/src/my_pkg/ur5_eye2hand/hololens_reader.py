#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState

import numpy as np

class HololensPosition():
    def __init__(self, name = "hololens_info_subscriber"):
        self.name = name
        self.pos = []
        self.k_pos = 0
        self.joint3pos = np.array([0,0,0])
        self.joint4pos = np.array([0,0,0])
        print("class is intialized!")
        

    def Init_node(self):
        rospy.init_node(self.name)
        sub = rospy.Subscriber("UnityJointStatePublish", JointState, self.callback)
        print('Initialization done!')
        return sub

    def Init_node2(self):
        rospy.init_node(self.name)
        sub = rospy.Subscriber("UnityJoint3ManipPublish", PointStamped, self.callback2)
        print('Initialization done!')
        return sub

    def callback(self, msg):
        d = list(msg.position)
        self.pos = d
        self.k_pos=msg.header.seq

    def callback2(self, msg):
        joint3_move = np.array([msg.point.x,msg.point.y,msg.point.z])
        self.joint3pos = joint3_move
        # rospy.loginfo(self.joint3pos)

    def callback3(self, msg):
        joint4_move = np.array([msg.point.x,msg.point.y,msg.point.z])
        self.joint4pos = joint4_move
        # rospy.loginfo(self.joint4pos)

if __name__ == "__main__":
    # hololens_info_reader = HololensPosition()   
    # hololens_info_reader.Init_node2()
    # while not rospy.is_shutdown():
    #     # print(vision_info_reader.pos)
    #     pass
    pass