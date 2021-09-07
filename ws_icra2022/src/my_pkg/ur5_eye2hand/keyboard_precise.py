#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
roslaunch my_pkg ur5_bringup.launch 
turn on external program on teach pendant!!
roslaunch pylon_camera pylon_camera_node.launch
roslaunch aruco_ros single_pylon.launch 
rosrun my_pkg ibvs_eye2hand_keyboard.py
yxj 20210826
'''
import rospy
from std_msgs.msg import String
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from ur5_pose_get import *
from vision_pose_get import VisionPosition
from hololens_reader import HololensPosition
import numpy as np
import matplotlib.pyplot as plt

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

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
    # rospy.loginfo(ss)
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
    dir = '/keyboard_precise/'
    current_path = os.path.dirname(__file__)

    rospy.init_node("move_ur5_by_urscript")
    pub=rospy.Publisher("/ur_hardware_interface/script_command",String,queue_size=10) # the subscriber is in ur5_bringup.launch

    # helolens_sub = rospy.Subscriber("/aruco_single/pixel", PointStamped, vision_reader.callback)
    
    ros_freq = 30
    rate=rospy.Rate(ros_freq)

    vision_reader = VisionPosition()
    vision_sub = rospy.Subscriber("/aruco_single/pixel", PointStamped, vision_reader.callback)
    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)
    # hololens_reader = HololensPosition()
    # sub = rospy.Subscriber("UnityJointStatePublish", JointState, hololens_reader.callback)

    pos1_flag = 0
    pos2_flag = 0
    flag_has_sent = 0
    flag_initialized = 0
    # initial_state=[0,-1.57,0,0,3.14,0.25]
    # initial_state=[-3.75,-89.27,-88.4,-90,90,1.34]# 单位是角度deg
    initial_state=[155.93,-123.81, -75.73, 1.97, 68.68, 147.01]# 单位是角度deg
    initial_state=change_angle_to_pi(initial_state)# 单位变成弧度rad

    # camera intrinsic parameters
    projection_matrix = np.reshape(np.array([2340.00415,    0.     ,  753.47694,    0.     ,\
            0.     , 2342.75146,  551.78795,    0.     ,\
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
    log_dqdot =  np.empty([6,0],float)
    v = [0,0,0,0,0,0]
    a = 50
    tt = 1

    dx = np.reshape([720,540],[2,1])

    dr = np.reshape([0.476,-0.140, 0.451],[3,1])
    drdot = np.zeros([3,1])
    
    Kp = 2*np.eye(2)
    Cd = np.eye(6)


    time.sleep(0.3)# wait for a short time otherwise q_last is empty
    q_last=ur_reader.now_ur_pos
    x_last = vision_reader.pos
    k_last = vision_reader.k_pos
    # k_last_h = hololens_reader.k_pos
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

        # k_now_h = hololens_reader.k_pos
        # d = hololens_reader.pos
        
        # 计算雅可比矩阵 J
        # urdf =  "/home/roboticslab/ur_ws/src/my_pkg/urdf/ur5.urdf"
        urdf =  current_path+"/../urdf/ur5.urdf"
        J,pose = get_jacobian_from_joint(urdf,q_now,0)
        J_inv = np.linalg.pinv(J)
        J_ori_inv = np.linalg.pinv(J[3:6,:]) # only compute orientation!!
        J_pos_inv = np.linalg.pinv(J[0:3,:])
        # print(J_inv)
        N = np.eye(6)-np.dot(J_inv,J)
        N_ori = np.eye(6)-np.dot(J_ori_inv,J[3:6,:])
        N_pos = np.eye(6)-np.dot(J_pos_inv,J[0:3,:])


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
            # 计算视觉矩阵Js
            u = x[0]-u0
            v = x[1]-v0
            z = 1 # =========================
            Js = np.array([      [fx/z, 0, u/z]     , [0, fy/z, v/z]    ])
            RR = np.array([[-7.34639719e-01, -6.27919077e-04,  6.78457138e-01],\
                                            [-6.78432812e-01,  9.19848654e-03, -7.34604865e-01],\
                                            [-5.77950645e-03, -9.99957496e-01, -7.18357448e-03]]) # 相机相对于base的旋转矩阵,要转置成base相对于相机才对
            Js = np.dot(Js,RR.T)
            # print(Js)
            Js_inv = np.linalg.pinv(Js)

            # 计算末端位置
            r4 = np.dot(pose, [[0],[0],[0],[1]])
            r = r4[0:3]
            # print('r is:',r)

            # 计算末端速度
            rdot = np.dot(J , np.reshape(qdot,[6,1]))
            # print(rdot)

            # 计算ut
            x = np.reshape(x,[2,1])
            ut = -np.dot( J_pos_inv, np.dot( Js_inv , np.dot(Kp, (x-dx) ) ) )
            # v = np.dot(N_ori,v)

            # 计算un
            if t-t_ready>15 and t-t_ready<16:
                d = np.reshape(np.array([-0.2,-0.2,0.2,0.2,0.2,0.1],float),[6,1]  )
                un = -np.dot(N_pos, np.dot(np.linalg.inv(Cd), d)  ) 
                # print(un)
            else:
                un = np.zeros([6,1])
            v = ut+un
            v = np.reshape(np.array(v),[-1,])

            # print('v',v.tolist())

            log_x.append(x.tolist())
            log_q = np.concatenate((log_q,np.reshape(q_now,[6,1])),axis=1)
            log_qdot = np.concatenate((log_qdot,np.reshape(qdot,[6,1])),axis=1)
            log_dqdot = np.concatenate((log_dqdot,np.reshape(v,[6,1])),axis=1)
            log_rdot = np.concatenate((log_rdot,np.reshape(rdot,[6,1])),axis=1)

            # 保护
            v[v>0.5]=0.5
            v[v<-0.5]=-0.5

            urscript_speedj_pub(pub,v,a,0.1)
            
            # if k_now != k_last:
            #     urscript_speedj_pub(pub,v,a,0.1)
            # else:
            #     urscript_speedj_pub(pub,[0,0,0,0,0,0],a,0.1)

        q_last = q_now
        k_last = k_now
        # k_last_h = k_now_h
        x_last = x_now
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
    np.save(current_path+dir+'log_dqdot.npy',log_dqdot)

    # task space velocity==============================================
    plt.figure(figsize=(30,20))
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('task space velocity %d' % (j+1),fontsize=20)
        plt.xlabel('time (s)')
        if j<3:
            plt.ylabel('velocity (m/s)')
        else:
            plt.ylabel('angular velocity (rad/s)')

        plt.plot(np.linspace(0,np.shape(log_rdot)[1]/ros_freq,np.shape(log_rdot)[1]),np.reshape(np.array(log_rdot[j,:]),[-1,]) ,label = 'actual veloc')
        plt.legend()
    plt.savefig(current_path+ dir+'log_r.jpg')

    # vision space position===============================================
    plt.figure()
    plt.plot(log_x_array[:,0], log_x_array[:,1],label = 'actual')
    plt.scatter(dx[0],dx[1],label = 'target', c='r')
    plt.legend()
    plt.title('vision space trajectory')
    plt.xlabel('x (pixel)')
    plt.ylabel('y (pixel)')
    plt.savefig(current_path+ dir+'log_x.jpg')

    # joint space velocity=================================================
    plt.figure(figsize=(30,20))
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('joint space velocity %d' % (j+1),fontsize=20)
        plt.xlabel('time (s)')
        plt.ylabel('velocity (m/s)')

        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),np.reshape(np.array(log_qdot[j,:]),[-1,]) ,label='actual joint velocity')
        plt.plot(np.linspace(0,np.shape(log_dqdot)[1]/ros_freq,np.shape(log_dqdot)[1]),log_dqdot[j,:].reshape(-1,), label = 'command joint velocity')
        plt.legend()
    plt.savefig(current_path+ dir+'log_qdot.jpg')

if __name__=="__main__":
    main()
