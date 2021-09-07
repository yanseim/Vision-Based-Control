# coding:utf-8
import cv2
import time

# cap = cv2.VideoCapture(1,cv2.CAP_DSHOW)
cap = cv2.VideoCapture(1)
time.sleep(1)
flag = cap.isOpened()
index = 1
while(flag):
    ret, frame = cap.read()
    cv2.imshow("image",frame)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('s'):
        cv2.imwrite(str(index)+".jpg",frame)
        print("save"+str(index)+".jpg successfully!!")
        index+=1
    elif k==ord('q'):
        break
cap.release()
cv2.destroyAllWindows()