#!/usr/bin/env python
# -*- coding: utf-8 -*-

from yaml.events import NodeEvent
import cv_bridge
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time

class ImageShow(object):
    def __init__(self):
        self.codec = cv2.VideoWriter_fourcc('M', 'P', 'E', 'G')
        self.fps = 30
        self.frameSize = (1440,1080)
        self.current_path = os.path.dirname(__file__)
        time_tuple = time.localtime(time.time())
        name=rospy.get_param("name")
        self.output = cv2.VideoWriter(self.current_path+"/Video/record_"+name+str(time_tuple[2])+"_"+str(time_tuple[3])+"_"+str(time_tuple[4])+".avi",self.codec, self.fps, self.frameSize)

        self.bridge = CvBridge()
        rospy.Subscriber('/aruco_single/result',Image,self.callback)

        self.img = None

        rospy.on_shutdown(self.shutDown)

    # ========================================================
    def callback(self,data):
        self.img = self.bridge.imgmsg_to_cv2(data,"bgr8")

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