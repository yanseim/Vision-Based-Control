#!/usr/bin/env python
# -*- coding: utf-8 -*-
from numpy.core.fromnumeric import reshape, shape
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
def urscript_speedj_pub1(pub , vel, ace,t):
    ss = "speedj([" + str(vel[0]) + "," + str(vel[1]) + "," + str(vel[2]) + "," + str(vel[3]) + "," + str(
        vel[4]) + "," + str(vel[5]) + "]," + "a=" + str(ace) + "," + "t=" + str(t) + ")"
    rospy.loginfo(ss)
    pub.publish(ss)
def urscript_speedj_pub(pub , vel, ace):
    ss = "speedj([" + str(vel[0]) + "," + str(vel[1]) + "," + str(vel[2]) + "," + str(vel[3]) + "," + str(
        vel[4]) + "," + str(vel[5]) + "]," + "a=" + str(ace) + ")"
    rospy.loginfo(ss)
    pub.publish(ss)


"""
rosrun rosserial_python serial_node.py /dev/ttyUSB0
rostopic pub /toggle_led std_msgs/Empty "{}" --once
"""
# class get_q():
#     def __init__(self,name = "q_subscriber"):
#         rospy.init_node(self.name)
#         self.q = []
#         sub = rospy.Subscriber("/joint_states", JointState, self.callback)
#         return sub

#     def callback(self, msg):
#         self.q = 
#         rospy.loginfo()

def main():
    rospy.init_node("move_ur5_by_urscript")
    pub=rospy.Publisher("/ur_hardware_interface/script_command",String,queue_size=10)
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
    log_q = np.empty([6,0],float)
    qdot = np.zeros([6,1],float)
    log_qdot = np.empty((6,0),float)
    log_dqdot =  np.empty((6,0),float)
    v = np.array([0.2,0.2,0.2,0.2,-0.2,0.2])# 往下
    # v = np.array([-0.05,-0.05,-0.05,-0.05,0.05,-0.05]) # 往上
    a = 50

    time.sleep(0.3)
    q_last=ur_reader.now_ur_pos
    t_start = time.time()
    t_last = t_start
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
        log_q = np.concatenate((log_q,np.reshape(q_now,[6,1])),axis=1)
        log_qdot = np.concatenate((log_qdot,np.reshape(qdot,[6,1])),axis=1)
        log_dqdot = np.concatenate((log_dqdot,np.reshape(v,[6,1])),axis=1)

        # if flag_initialized==0:
        #     moveur(pub,initial_state,5,0.3,2)
        #     flag_initialized=1
        # elif flag_initialized==1 and t-t_start>2.2 and  all(np.abs(np.array(q_now)-np.array(initial_state)))<1e-5:
        #     flag_initialized=2
        #     print("initialization is done! the time is:", t-t_start)
        # elif flag_initialized==2  and t-t_start<3:
        #     urscript_speedj_pub1(pub,v,a,0)
        # elif flag_initialized==2  and t-t_start<5 and t-t_start>=3:
        #     v =[-0.05,-0.05,-0.05,-0.05,0.05,-0.05]
        #     urscript_speedj_pub1(pub,v,a,0)

        # if flag_initialized==0 and t-t_start<3:
        #     moveur(pub,initial_state,30,0.3,0)
        #     time.sleep(0.5)

        if flag_initialized==0 and t-t_start<3:
            print("the time is : ",t-t_start)
            urscript_speedj_pub1(pub,v,a,1)
            print("the time is : ",t-t_start)
            flag_initialized = 1
        elif flag_initialized==1:
            print("the time is : ",t-t_start)
            v = -np.array(v)
            urscript_speedj_pub1(pub,v,a,1)
            print("the time is : ",t-t_start)
            flag_initialized=2


        time.sleep(0.5)

        # rospy.loginfo(q_now)
        q_last = q_now
        t_last = t
        # time.sleep(2)

        rate.sleep()
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
