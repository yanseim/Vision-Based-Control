import cv2
import numpy as np
 
# 读取图片并缩放方便显示
img = cv2.imread('image2.png')
height, width = img.shape[:2]
# size = (int(width * 0.2), int(height * 0.2))
size = (int(width), int(height))
log = np.empty([0,3],float)

# 缩放
img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
 
# BGR转化为HSV
HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
 
# 鼠标点击响应事件
def getposHsv(event, x, y, flags, param):
    global log
    if event==cv2.EVENT_LBUTTONDOWN:
        print("HSV is", HSV[y, x])
        log = np.concatenate((log,np.reshape(np.array(HSV[y, x]),[1,3])),axis=0)
 
 
def getposBgr(event, x, y, flags, param):
    if event==cv2.EVENT_LBUTTONDOWN:
        print("Bgr is", img[y, x])
 
if __name__=="__main__":
    cv2.imshow("imageHSV", HSV)
    # cv2.imshow('image', img)
    cv2.setMouseCallback("imageHSV", getposHsv)
    # cv2.setMouseCallback("image", getposBgr)

    k = cv2.waitKey(0) & 0xFF
    if k == ord('s'):
        min = [np.min(log[:,0]),np.min(log[:,1]),np.min(log[:,2])]
        max = [np.max(log[:,0]),np.max(log[:,1]),np.max(log[:,2])]
        print(log)
        print(min)
        print(max)

    cv2.waitKey(0)