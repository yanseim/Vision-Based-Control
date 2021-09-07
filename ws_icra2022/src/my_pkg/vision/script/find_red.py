#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import time
import numpy as np
from std_msgs.msg import UInt8

class DetectRed:

    def __init__(self):
        self.obj_buf = []
        self.desire_buf = []

        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.bgr_image=None

    def callback(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8") # opencv默认BGR格式
            self.bgr_image = video_capture.copy()
            self.process_publish_bgr_image(self.bgr_image)
        except CvBridgeError as e:
            print(e)

    def convert_hsv(self,image):
        # bgr=image[...,::-1]
        return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    def select_hsv_red(self,image):
        # mask1
        lower = np.array([170, 189, 149])
        upper = np.array([180, 220, 239])

        red_mask1 = cv2.inRange(image, lower, upper)

        # # mask2
        # lower = np.uint8([140, 12, 77])
        # upper = np.uint8([179, 241, 255])
        # red_mask2 = cv2.inRange(image, lower, upper)
        # combine the mask
        # mask = cv2.bitwise_or(red_mask1, red_mask2)
        mask = red_mask1
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)))
        masked = cv2.bitwise_and(image, image, mask=mask)
        return mask,masked

    def calculate_coordinate(self,mask):
        img,contours, hierarchy  = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours)>=1:
            moment = cv2.moments(contours[0])
            x = int(moment["m10"]/moment["m00"])
            y = int(moment["m01"]/moment["m00"])
            flag = 1
        else:
            flag = 0
            x = 0
            y = 0
        return flag, (x,y)
        
    def process_publish_bgr_image(self,bgr_image):
        if bgr_image is not None:
            hsv_image = self.convert_hsv(bgr_image)
            
            mask, masked_image_hsv = self.select_hsv_red(hsv_image)
            # cv2.imshow("mask",mask)
            # cv2.waitKey()
            flag, coordinate = self.calculate_coordinate(mask)

            masked_image_bgr = cv2.cvtColor(masked_image_hsv, cv2.COLOR_HSV2BGR)
            if flag==1:
                rospy.loginfo(coordinate)
                cv2.circle(masked_image_bgr, coordinate,7,128,-1)
            else:
                rospy.loginfo("cannot find object!!!")
            # 再将opencv格式额数据转换成ros image格式的数据发布
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(masked_image_bgr, "bgr8"))
            except CvBridgeError as e:
                print(e)


# def convert_hsv(image):
#     # bgr=image[...,::-1]
#     return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# def select_hsv_red(image):
#     # mask1
#     lower = np.array([170, 189, 149])
#     upper = np.array([180, 220, 239])

#     red_mask1 = cv2.inRange(image, lower, upper)

#     # # mask2
#     # lower = np.uint8([140, 12, 77])
#     # upper = np.uint8([179, 241, 255])
#     # red_mask2 = cv2.inRange(image, lower, upper)
#     # combine the mask
#     # mask = cv2.bitwise_or(red_mask1, red_mask2)
#     mask = red_mask1
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)))
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)))
#     masked = cv2.bitwise_and(image, image, mask=mask)
#     return mask,masked

# def calculate_coordinate(mask):
#     img,contours, hierarchy  = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     if len(contours)>=1:
#         moment = cv2.moments(contours[0])
#         x = int(moment["m10"]/moment["m00"])
#         y = int(moment["m01"]/moment["m00"])
#         flag = 1
#     else:
#         flag = 0
#         x = 0
#         y = 0
#     return flag, (x,y)

# def process_bgr_object_image(bgr_image):
#     if bgr_image is not None:
#         hsv_image = convert_hsv(bgr_image)

        
#         mask, masked_image_hsv = select_hsv_red(hsv_image)

#         flag, coordinate = calculate_coordinate(mask)

#         masked_image_bgr = cv2.cvtColor(masked_image_hsv, cv2.COLOR_HSV2BGR)
#         if flag==1:
#             print(coordinate)
#             cv2.circle(masked_image_bgr, coordinate,7,128,-1)
#         cv2.imshow("after calculate",masked_image_bgr)
#         cv2.waitKey(0)
#         # 再将opencv格式额数据转换成ros image格式的数据发布



def main():
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        k=DetectRed()
        # flag_data = code_flags() 
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # k.process_bgr_object_image(k.bgr_image)

            rate.sleep()
            # except:
            #     print "no sucking tile----"
       # rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()

if __name__=="__main__":
    main()

    cv2.waitKey()