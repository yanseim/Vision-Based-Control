#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
roslaunch my_pkg ur5_bringup.launch 
rosrun my_pkg test_speedj_sin.py

yxj 20210706
'''


import rospy
from rospy.rostime import Duration
from std_msgs.msg import String
import time
from sensor_msgs.msg import JointState
import socket
from ur5_pose_get import *
import numpy as np
import matplotlib.pyplot as plt
import math


def change_angle_to_pi(qangle):
    temp=[]
    for i in range(len(qangle)):
        temp.append(qangle[i]/180.0*3.14)
    return temp
def moveur(pub,q,ace,vel,t):
    # ss="movej(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    ss="movej(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+")"
    rospy.loginfo(ss)
    pub.publish(ss)
def movelur(pub,q,ace,vel,t):
    ss="movel(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    rospy.loginfo(ss)
    pub.publish(ss)
def movecur(pub,q,ace,vel,t):
    ss="movec(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    rospy.loginfo(ss)
    pub.publish(ss)
def urscript_speedj_pub(pub , vel, ace,t): # t是必须要有的，规定了动作时间，结束后速度会降为0，所以持续时间一定要比指令周期长
    ss = "speedj([" + str(vel[0]) + "," + str(vel[1]) + "," + str(vel[2]) + "," + str(vel[3]) + "," + str(
        vel[4]) + "," + str(vel[5]) + "]," + "a=" + str(ace) + "," + "t=" + str(t) + ")"
    rospy.loginfo(ss)
    pub.publish(ss)



def main():
    rospy.init_node("move_ur5_by_urscript")
    pub=rospy.Publisher("/ur_hardware_interface/script_command",String,queue_size=10) # the subscriber is in ur5_bringup.launch
    ros_freq = 50
    rate=rospy.Rate(ros_freq)

    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)

    pos1_flag = 0
    pos2_flag = 0
    flag_has_sent = 0
    flag_initialized = 0
    initial_state=[0,-1.57,0,0,3.14,0.25]

    # prepare for speedj
    qdot = np.zeros([6,1],float)
    log_q = np.empty([6,0],float)
    log_qdot = np.empty([6,0],float)
    log_dqdot =  np.empty([6,0],float)
    v = [0,0,0,0,0,0]
    a = 50
    tt = 1

    time.sleep(0.3)# wait for a short time otherwise q_last is empty
    q_last=ur_reader.now_ur_pos
    t_start = time.time()
    t_last = t_start
    t_ready = t_start
    print("time begins at: ",t_start)
    # ================================================while begins
    while not rospy.is_shutdown():
        rospy.loginfo("start while-----")

        t = time.time()
        q_now=ur_reader.now_ur_pos
        if len(q_now)==0:
            print("can not get q !!!!!")
            continue
        
        for ii in range(6):
            qdot[ii]=(q_now[ii]-q_last[ii])*ros_freq
        
        # print('qdot is: ',qdot)

        if flag_initialized==0:# 先回到初始位置，movej的t参数是不需要的，这里给0
            moveur(pub,initial_state,5,0.3,0)
            time.sleep(5)
            flag_initialized=1
        elif flag_initialized==1 and t-t_start>5 and  all(np.abs(np.array(q_now)-np.array(initial_state)))<1e-5:# 判断是否到达
            flag_initialized=2
            print("initialization is done! the time is:", t-t_start)
            t_ready = t
        elif flag_initialized==2  and t-t_start<20:# 开始按sin的速度运动 并记录
            v = [-0.2*math.sin(math.pi*(t-t_ready)),-0.2*math.sin(math.pi*(t-t_ready)),-0.2*math.sin(math.pi*(t-t_ready)),-0.2*math.sin(math.pi*(t-t_ready)),0.2*math.sin(math.pi*(t-t_ready)),-0.2*math.sin(math.pi*(t-t_ready))] # 往上
            log_q = np.concatenate((log_q,np.reshape(q_now,[6,1])),axis=1)
            log_qdot = np.concatenate((log_qdot,np.reshape(qdot,[6,1])),axis=1)
            log_dqdot = np.concatenate((log_dqdot,np.reshape(v,[6,1])),axis=1)
            urscript_speedj_pub(pub,v,a,1)

        q_last = q_now
        t_last = t

        rate.sleep()# Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called. 这里就是ros_freq Hz
    # ===============================================while ends
    print(time.time()-t_start)

    print(np.shape(log_qdot))
    
    np.save('log_q.npy',log_q)
    np.save('log_qdot',log_qdot)
    np.save('log_dqdot',log_dqdot)

    plt.figure(figsize=(30,20))
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('joint %d velocity' % (j+1),fontsize=20)
        plt.xlabel('time[s]')
        plt.ylabel('joint velocity[rad/s]')

        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),log_qdot[j,:])
        plt.plot(np.linspace(0,np.shape(log_dqdot)[1]/ros_freq,np.shape(log_dqdot)[1]),log_dqdot[j,:])
    plt.savefig(os.path.join('/home/roboticslab/ur_ws/src/my_pkg/ur5/', 'log_q.jpg'))

if __name__=="__main__":
    main()
