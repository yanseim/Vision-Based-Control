#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import rospy
from std_msgs.msg import String
import time

class UR_script():
    def __init__(self,name = "ur_subscriber",ros_freq = 50):
        self.name = name
        self.command = []
        self.ros_freq = ros_freq

        rospy.init_node(self.name)
        self.sub = rospy.Subscriber("/ur_driver/URScript", String, self.callback)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        HOST ="192.168.100.50" 
        PORT = 30003
        self.s.connect((HOST, PORT))
        print("connection succeed!")

    def callback(self, msg):
        self.s.send (str(msg.data)+"\n") # 没注意到这里要加一个.data
        rospy.loginfo(rospy.get_caller_id() +  str(msg.data))
        time.sleep(1.0/self.ros_freq)
        # self.rate.sleep() #好像不能用这个，不知道为什么
        

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() , data.data)
    
# def listener():
#     rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber("chatter", String, callback)
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

if __name__=="__main__":

    listen_send = UR_script()
    while not rospy.is_shutdown():
        w=1
    


