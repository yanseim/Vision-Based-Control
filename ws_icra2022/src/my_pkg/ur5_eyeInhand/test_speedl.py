#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
roslaunch my_pkg ur5_bringup.launch 
rosrun my_pkg test_speedl.py

yxj 20210728
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

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from ur5_kinematics import Kinematic

import os

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
def urscript_speedl_pub(pub , vel, ace,t): # xd, a, t
    ss = "speedl([" + str(vel[0]) + "," + str(vel[1]) + "," + str(vel[2]) + "," + str(vel[3]) + "," + str(
        vel[4]) + "," + str(vel[5]) + "]," + "a=" + str(ace) + "," + "t=" + str(t) + ")"
    rospy.loginfo(ss)
    pub.publish(ss)
def urscript_get_actual_tcp_speed_pub(pub):
    ss = "get_actual_tcp_speed()"
    rospy.loginfo(ss)
    pub.publish(ss)
# def get_jacobian_from_joint(urdfname,jointq,flag):
#     #robot = URDF.from_xml_file("/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
#     robot = URDF.from_xml_file(urdfname)
#     tree = kdl_tree_from_urdf_model(robot)
#     # print tree.getNrOfSegments()
#     chain = tree.getChain("base_link", "ee_link")
#     # print chain.getNrOfJoints()
#     # forwawrd kinematics
#     kdl_kin = KDLKinematics(robot, "base_link", "ee_link")
#     q=jointq
#     pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
#     q0=Kinematic()
#     # print q0.Forward(q)
#     if flag==1:
#         q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward(q))
#     else:
#         q_ik = kdl_kin.inverse(pose)  # inverse kinematics
#     # print "----------iverse-------------------\n", q_ik

#     if q_ik is not None:
#         pose_sol = kdl_kin.forward(q_ik)  # should equal pose
#         print "------------------forward ------------------\n",pose_sol
#     J = kdl_kin.jacobian(q)
#     return J,pose
def get_jacobian_from_joint(urdfname,jointq,flag):
    #robot = URDF.from_xml_file("/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
    robot = URDF.from_xml_file(urdfname)
    tree = kdl_tree_from_urdf_model(robot)
    # print tree.getNrOfSegments()
    chain = tree.getChain("base_link", "tool0")
    # print chain.getNrOfJoints()
    # forwawrd kinematics
    kdl_kin = KDLKinematics(robot, "base_link", "tool0")
    q=jointq
    #q = [0, 0, 1, 0, 1, 0]
    pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)

    J = kdl_kin.jacobian(q)
    #print 'J:', J
    return J,pose

def main():
    dir = '/fig_speedl/'
    current_path = os.path.dirname(__file__)

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
    # initial_state=[0,-1.57,0,0,3.14,0.25]
    # initial_state=[-3.75,-89.27,-88.4,-90,90,1.34]# 单位是角度deg
    initial_state=[-3.75,-80,-85,-90,90,1.34]# 单位是角度deg
    initial_state=change_angle_to_pi(initial_state)# 单位变成弧度rad

    # prepare for speedl
    qdot = np.zeros([6,1],float)
    log_q = np.empty([6,0],float)
    log_qdot = np.empty([6,0],float)
    log_rdot = np.empty([6,0],float)
    log_drdot =  np.empty([6,0],float)
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
        qdot_now = ur_reader.now_joint_vel
        if len(q_now)==0:
            print("can not get q !!!!!")
            continue

        # 计算雅可比矩阵
        urdf =  "/home/roboticslab/ur_ws/src/my_pkg/urdf/ur5.urdf"
        J,pose = get_jacobian_from_joint(urdf,q_now,0)
        print(J)

        # # 计算角速度
        # for ii in range(6):
        #     qdot[ii]=(q_now[ii]-q_last[ii])*ros_freq
        qdot = qdot_now
        
        print('qdot is: ',qdot)

        # 计算末端速度
        rdot = np.dot(J , np.reshape(qdot,[6,1]))
        rdot[0],rdot[1],rdot[3],rdot[4]  = -rdot[0],-rdot[1],-rdot[3],-rdot[4] 
        print(rdot)

        # 给指令
        if flag_initialized==0:# 先回到初始位置，movej的t参数是不需要的，这里给0
            moveur(pub,initial_state,5,0.3,0)
            time.sleep(5)
            flag_initialized=1
        elif flag_initialized==1:
            flag_initialized=2
            print("initialization is done! the time is:", t-t_start)
            t_ready = t
        elif flag_initialized==2  and t-t_ready<10:# 开始按sin的速度运动 并记录
            # v = [-0.1*math.sin(math.pi*(t-t_ready)),-0.1*math.sin(math.pi*(t-t_ready)),-0.1*math.sin(math.pi*(t-t_ready)),\
            #     0,0,0]
            v = [0,0,0,0,0.2,0]
            flag_initialized=3
            urscript_speedl_pub(pub,v,a,1)
        elif flag_initialized==3:
            log_q = np.concatenate((log_q,np.reshape(q_now,[6,1])),axis=1)
            log_qdot = np.concatenate((log_qdot,np.reshape(qdot,[6,1])),axis=1)
            log_drdot = np.concatenate((log_drdot,np.reshape(v,[6,1])),axis=1)
            log_rdot = np.concatenate((log_rdot,np.reshape(rdot,[6,1])),axis=1)

        q_last = q_now
        t_last = t

        rate.sleep()# Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called. 这里就是ros_freq Hz
    # ===============================================while ends
    print(time.time()-t_start)

    print(np.shape(log_qdot))
    
    np.save(current_path+ dir+'log_q.npy',log_q)
    np.save(current_path+dir+'log_qdot.npy',log_qdot)
    np.save(current_path+dir+'log_rdot.npy',log_rdot)
    np.save(current_path+dir+'log_drdot.npy',log_drdot)

    plt.figure(figsize=(30,20))
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('task space velocity %d' % (j+1),fontsize=20)
        plt.xlabel('time[s]')
        plt.ylabel('task space velocity')

        plt.plot(np.linspace(0,np.shape(log_rdot)[1]/ros_freq,np.shape(log_rdot)[1]).reshape(1,-1),log_rdot[j,:].reshape(1,-1))# 不知道为什么这个给我报错，reshape(-1)明明是正确的
        plt.plot(np.linspace(0,np.shape(log_drdot)[1]/ros_freq,np.shape(log_drdot)[1]).reshape(1,-1),log_drdot[j,:].reshape(1,-1))
    plt.savefig(os.path.join('/home/roboticslab/ur_ws/src/my_pkg/ur5/fig_speedl/', 'log_r.jpg'))

if __name__=="__main__":
    main()
