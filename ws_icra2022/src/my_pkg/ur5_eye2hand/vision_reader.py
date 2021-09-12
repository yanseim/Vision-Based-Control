#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3Stamped

class VisionPosition():
    def __init__(self, name = "hololens_info_subscriber"):
        self.name = name
        self.pos = []
        self.actual_z = 0.0
        self.k_pos = 0

    def Init_node(self):
        rospy.init_node(self.name)
        sub = rospy.Subscriber("/aruco_single/pixel", PointStamped, self.callback)
        actual_z_sub = rospy.Subscriber("/aruco_single/position", Vector3Stamped, self.callback_actual_z)
        return sub

    def callback(self, msg):
        # print([msg.point.x,msg.point.y])
        self.pos = list([msg.point.x,msg.point.y])
        self.k_pos=msg.header.seq

    def callback_actual_z(self,msg):
        self.actual_z = msg.vector.z
        print(self.actual_z)

if __name__ == "__main__":
    vision_info_reader = VisionPosition()
    vision_info_reader.Init_node()
    while not rospy.is_shutdown():
        # print(vision_info_reader.pos)
        pass
