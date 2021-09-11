#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time
from geometry_msgs.msg import PointStamped

class ImageShow(object):
    def __init__(self):
        self.codec = cv2.VideoWriter_fourcc('M', 'P', 'E', 'G')
        self.fps = 30
        self.frameSize = (1440,1080)
        self.current_path = os.path.dirname(__file__)
        time_tuple = time.localtime(time.time())
        self.name=rospy.get_param("name")
        # self.pos = []
        self.output = cv2.VideoWriter(self.current_path+"/Video/record_"+self.name+str(time_tuple[2])+"_"+str(time_tuple[3])+"_"+str(time_tuple[4])+".avi",self.codec, self.fps, self.frameSize)
        self.circle_list = []
        self.bridge = CvBridge()
        rospy.Subscriber('/aruco_single/result',Image,self.callback)
        rospy.Subscriber('/aruco_single/pixel',PointStamped,self.callback2)

        self.img = None

        rospy.on_shutdown(self.shutDown)

    # ========================================================
    def callback(self,data):
        self.img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        if self.name =="point":
            cv2.circle(self.img, (720,540), 40,(0,0,255),-1)
        elif self.name =="circle":
            # self.circle_list.append(self.pos)
            # for center in self.circle_list:
            #     cv2.circle(self.img,center, 30,(0,0,255),-1)
            cv2.circle(self.img,(720,540), 100,(0,0,255),thickness=10)

    def callback2(self,data):
        self.pos = (int(data.point.x),int(data.point.y))

    # ========================================================
    def shutDown(self):
        self.output.release()
        cv2.destroyAllWindows()

    # ========================================================
    def main(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.img is None:
                continue
            img_show = self.img.copy()
            self.output.write(img_show )
            cv2.imshow("frame", img_show)
            cv2.waitKey(1)
            rate.sleep()


if __name__=='__main__':
    try:
        rospy.init_node("image_show_node")
        video = ImageShow()
        video.main()
    except rospy.ROSInterruptException:
        pass