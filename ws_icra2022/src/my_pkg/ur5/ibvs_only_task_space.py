#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
roslaunch my_pkg ur5_bringup.launch 
turn on external program on teach pendant!!
roslaunch usb_cam usb_cam-test.launch
roslaunch my_pkg single.launch 
rosrun my_pkg ibvs.py
yxj 20210826
'''

from numpy import linalg
import rospy
from rospy.rostime import Duration
from std_msgs.msg import String
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import socket
from ur5_pose_get import *
from vision_pose_get import VisionPosition
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

def get_jacobian_from_joint(urdfname,jointq,flag):
    #robot = URDF.from_xml_file("/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
    robot = URDF.from_xml_file(urdfname)
    tree = kdl_tree_from_urdf_model(robot)
    # print tree.getNrOfSegments()
    chain = tree.getChain("base_link", "wrist_3_link")
    # print chain.getNrOfJoints()
    # forwawrd kinematics
    kdl_kin = KDLKinematics(robot, "base_link", "wrist_3_link")
    q=jointq
    #q = [0, 0, 1, 0, 1, 0]
    pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
    pose[0,:],pose[1,:] = -pose[0,:],-pose[1,:]# added on 0728 yxj
    # 注意这里是从base_link到wrist_3_link。调外参的时候是base到wrist_3_link，相差z轴转180度。这里转了之后就相当于从base开始了 0826 yxj

    J = kdl_kin.jacobian(q)
    J[0,:],J[1,:],J[3,:],J[4,:]  = -J[0,:],-J[1,:],-J[3,:],-J[4,:] # added on 0728 yxj
    #print 'J:', J
    return J,pose

def main():
    dir = '/fig_ibvs/'
    current_path = os.path.dirname(__file__)

    rospy.init_node("move_ur5_by_urscript")
    pub=rospy.Publisher("/ur_hardware_interface/script_command",String,queue_size=10) # the subscriber is in ur5_bringup.launch
    ros_freq = 30
    rate=rospy.Rate(ros_freq)

    vision_reader = VisionPosition()
    vision_sub = rospy.Subscriber("/aruco_single/pixel", PointStamped, vision_reader.callback)
    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)

    pos1_flag = 0
    pos2_flag = 0
    flag_has_sent = 0
    flag_initialized = 0
    # initial_state=[0,-1.57,0,0,3.14,0.25]
    # initial_state=[-3.75,-89.27,-88.4,-90,90,1.34]# 单位是角度deg
    initial_state=[147.73,-81.12, -97.04, -79.13, 87.39, -155.48]# 单位是角度deg
    initial_state=change_angle_to_pi(initial_state)# 单位变成弧度rad

    # camera intrinsic parameters
    projection_matrix = np.reshape(np.array([ 688.84155,    0.     ,  319.01795,    0.     ,\
                                                0.     ,  688.81415,  232.94001,    0.     ,\
                                                0.     ,    0.     ,    1.     ,    0.     ]),[3,4] )
    fx = projection_matrix[0,0]
    fy = projection_matrix[1,1]
    u0 = projection_matrix[0,2]
    v0 = projection_matrix[1,2]

    # prepare for logging data
    qdot = np.zeros([6,1],float)
    log_x = []
    log_q = np.empty([6,0],float)
    log_qdot = np.empty([6,0],float)
    log_rdot = np.empty([6,0],float)
    log_drdot =  np.empty([6,0],float)
    v = [0,0,0,0,0,0]
    a = 50
    tt = 1

    dx = np.reshape([320,240],[2,1])

    dr = np.reshape([0.476,-0.140, 0.451],[3,1])
    drdot = np.zeros([3,1])
    Kp = 2*np.eye(2)


    time.sleep(0.3)# wait for a short time otherwise q_last is empty
    q_last=ur_reader.now_ur_pos
    x_last = vision_reader.pos
    k_last = vision_reader.k_pos
    t_start = time.time()
    t_last = t_start
    t_ready = t_start
    print("time begins at: ",t_start)
    # ================================================while begins
    while not rospy.is_shutdown():
        # rospy.loginfo("start while-----")

        t = time.time()
        q_now=ur_reader.now_ur_pos
        qdot = ur_reader.now_joint_vel
        
        if len(q_now)==0:
            print("can not get q !!!!!")
            continue
        x_now = vision_reader.pos
        x = x_now
        k_now = vision_reader.k_pos
        
        # 计算雅可比矩阵 J
        # urdf =  "/home/roboticslab/ur_ws/src/my_pkg/urdf/ur5.urdf"
        urdf =  current_path+"/../urdf/ur5.urdf"
        J,pose = get_jacobian_from_joint(urdf,q_now,0)
        J_inv = np.linalg.pinv(J)
        J_ori_inv = np.linalg.pinv(J[3:6,:]) # only compute orientation!!
        # print(J_inv)
        N_ori = np.eye(6)-np.dot(J_ori_inv,J[3:6,:])

        # 计算视觉矩阵Js
        u = x[0]-u0
        v = x[1]-v0
        z = 0.5 # =========================
        Js = np.array([      [-fx/z, 0, u/z, u*v/fx, -(fx*fx+u*u)/fx, u]     , [0, -fy/z, v/z, (fy*fy+v*v)/fy, -u*v/fy, -v]    ])
        # print(Js)
        R_extrinsic = np.array([[-0.76111026 ,0.64717796,-0.043265  ],\
         [-0.58715626,-0.65910701, 0.46992071],\
         [ 0.27560606, 0.38306479, 0.8816477 ]])
        # print(R_extrinsic)
        R = np.dot(R_extrinsic, pose[0:3,0:3])# 相机相对于base的旋转矩阵
        # print(R)
        RR = np.zeros([6,6])
        RR[0:3,0:3] = R
        RR[3:6,3:6] = R
        # print(RR)
        Js = np.dot(Js,RR)
        # print(Js)
        Js_inv = np.linalg.pinv(Js)

        # 计算末端位置
        r4 = np.dot(pose, [[0],[0],[0],[1]])
        r = r4[0:3]
        # print('r is:',r)

        # 计算末端速度
        rdot = np.dot(J , np.reshape(qdot,[6,1]))
        # print(rdot)

        # 给指令
        if flag_initialized==0:# 先回到初始位置，movej的t参数是不需要的，这里给0
            moveur(pub,initial_state,5,0.3,0)
            time.sleep(5)
            flag_initialized=1
        elif flag_initialized==1:
            flag_initialized=2
            print("initialization is done! the time is:", t-t_start)
            t_ready = t
        elif flag_initialized==2  and t-t_ready<20:# 
            x = np.reshape(x,[2,1])
            v = -np.dot( J_inv, np.dot( Js_inv , np.dot(Kp, (x-dx) ) ) )
            # v = np.dot(N_ori,v)
            v = np.reshape(np.array(v),[-1,])

            # print('v',v.tolist())

            log_x.append(x.tolist())
            log_q = np.concatenate((log_q,np.reshape(q_now,[6,1])),axis=1)
            log_qdot = np.concatenate((log_qdot,np.reshape(qdot,[6,1])),axis=1)
            log_drdot = np.concatenate((log_drdot,np.reshape(v,[6,1])),axis=1)
            log_rdot = np.concatenate((log_rdot,np.reshape(rdot,[6,1])),axis=1)

            # 保护
            v[v>0.5]=0.5
            v[v<-0.5]=-0.5
            
            if k_now != k_last:
                urscript_speedj_pub(pub,v,a,0.1)
            else:
                urscript_speedj_pub(pub,[0,0,0,0,0,0],a,0.1)

        q_last = q_now
        x_last = x_now
        k_last = k_now
        t_last = t

        rate.sleep()# Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called. 这里就是ros_freq Hz
    # ===============================================while ends
    print(time.time()-t_start)

    print(np.shape(log_qdot))
    log_x_array = np.array(log_x)

    np.save(current_path+ dir+'log_x.npy',log_x_array)
    np.save(current_path+ dir+'log_q.npy',log_q)
    np.save(current_path+dir+'log_qdot.npy',log_qdot)
    np.save(current_path+dir+'log_rdot.npy',log_rdot)
    np.save(current_path+dir+'log_drdot.npy',log_drdot)

    plt.figure(figsize=(30,20))
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('task space velocity %d' % (j+1),fontsize=20)
        plt.xlabel('time (s)')
        plt.ylabel('task space velocity')

        plt.plot(np.linspace(0,np.shape(log_rdot)[1]/ros_freq,np.shape(log_rdot)[1]),np.reshape(np.array(log_rdot[j,:]),[-1,]) )
        plt.plot(np.linspace(0,np.shape(log_drdot)[1]/ros_freq,np.shape(log_drdot)[1]),log_drdot[j,:].reshape(-1,))
    plt.savefig(current_path+dir+'log_r.jpg')

    plt.figure()
    plt.plot(log_x_array[:,0], log_x_array[:,1],label = 'actual')
    plt.scatter(dx[0],dx[1],label = 'target')
    plt.legend()
    plt.title('vision space trajectory')
    plt.xlabel('x (pixel)')
    plt.ylabel('y (pixel)')
    plt.savefig(current_path+dir+'log_x.jpg')

if __name__=="__main__":
    main()
