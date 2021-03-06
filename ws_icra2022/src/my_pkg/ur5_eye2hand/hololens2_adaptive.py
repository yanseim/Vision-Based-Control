#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
roslaunch my_pkg ur5_bringup.launch 
turn on external program on teach pendant!!
roslaunch pylon_camera pylon_camera_node.launch
roslaunch aruco_ros single_pylon.launch 
rosrun my_pkg ibvs_eye2hand_hololens.py
yxj 20210826
'''
import rospy
from std_msgs.msg import String
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3Stamped
from ur5_pose_get import *
from vision_pose_get import VisionPosition
from hololens_reader import HololensPosition
import numpy as np
import matplotlib.pyplot as plt

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

import os

import random

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

def get_jacobian_for_hololens(urdfname,jointq,flag):
    robot = URDF.from_xml_file(urdfname)
    kdl_kin = KDLKinematics(robot, "base_link", "forearm_link")
    J1 = kdl_kin.jacobian(jointq[:3])
    J1[0,:],J1[1,:],J1[3,:],J1[4,:]  = -J1[0,:],-J1[1,:],-J1[3,:],-J1[4,:] # added on 0728 yxj
    # print 'J:', J
    kdl_kin = KDLKinematics(robot, "base_link", "wrist_1_link")
    J2 = kdl_kin.jacobian(jointq[:4])
    J2[0,:],J2[1,:],J2[3,:],J2[4,:]  = -J2[0,:],-J2[1,:],-J2[3,:],-J2[4,:] # added on 0728 yxj
    return J1,J2

def main():
    time.sleep(0.8)

    current_path = os.path.dirname(__file__)    
    dir = '/hololens2_adaptive/0915_video/'
    urdf =  current_path+"/../urdf/ur5.urdf"

    rospy.init_node("move_ur5_by_urscript")
    pub=rospy.Publisher("/ur_hardware_interface/script_command",String,queue_size=10) # the subscriber is in ur5_bringup.launch

    # helolens_sub = rospy.Subscriber("/aruco_single/pixel", PointStamped, vision_reader.callback)
    
    ros_freq = 30
    rate=rospy.Rate(ros_freq)

    vision_reader = VisionPosition()
    vision_sub = rospy.Subscriber("/aruco_single/pixel", PointStamped, vision_reader.callback)
    actual_z_sub = rospy.Subscriber("/aruco_single/position", Vector3Stamped, vision_reader.callback_actual_z)
    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)
    hololens_reader = HololensPosition()
    sub = rospy.Subscriber("UnityJointStatePublish", JointState, hololens_reader.callback)
    sub2 = rospy.Subscriber("UnityJoint3ManipPublish", PointStamped, hololens_reader.callback2)
    sub3 = rospy.Subscriber("UnityJoint4ManipPublish", PointStamped, hololens_reader.callback3)

    pos1_flag = 0
    pos2_flag = 0
    flag_has_sent = 0
    flag_initialized = 0
    initial_state=[155.93,-123.81, -75.73, 1.97, 68.68, 147.01]# 单位是角度deg # 0904前
    # initial_state=[141.44,-96.23, -94.76, 19.88, 79.35, 146.93]# 单位是角度deg
    initial_state1=[106.34,-171.4, 6.67, 24.88, 86, 146.86]# 0907挪位置了=====================================
    initial_state1=[101.62,-143.69, -36.4, -22.64, 123.27, 146.86]# 0913夜12点
    initial_state1 = change_angle_to_pi(initial_state1)

    initial_state=[102.73,-135.07, -52.55, -29.65, 68.61, 146.93]# 0908 experiment1-
    initial_state=[101.62,-143.69, -36.4, -22.64, 123.27, 146.86]# 0913夜12点

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
    log_theta_z = []
    log_z_hat = []
    log_actual_z = []
    log_theta_k = []
    log_Js_hat = []
    log_Js_ref = []
    log_x = []
    log_d = []
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
    
    # Kp = 2*np.eye(2)
    # Cd = np.eye(6)
    # L_z = 0.00001*np.eye(13)
    # L_k = 0.00001*np.eye(15)# ======================步长是调好的，不能再大了
    Kp = 3*np.eye(2)
    Cd = np.eye(6)
    # L_z = 0.00003*np.eye(13)
    # L_k = 0.00001*np.eye(15)# ======================步长是调好的，不能再大了
    L_z = 0.0003*np.diag([0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,1,1,1,3])
    L_z = 0.0003*np.diag([0,0,0,0,0,0,0,0,0,1,1,1,3])
    L_k = 0.001*np.diag([1000,1000,1000,1000,1000,1000,0,0,0,10,10,10,10,10,10])
    # L_z = 0.0000*np.diag([1,1,1,1,1,1,1,1,1,1,1,1,1.2])
    # L_k = 0.000*np.diag([1000,1000,1000,1000,1000,1000,0,0,0,10,10,10,10,10,10])
    # L_z = 0.0003*np.diag([1,1,1,3])
    # L_k = 0.001*np.diag([1000,1000,1000,1000,1000,1000,0,0,0,10,10,10,10,10,10])

    time.sleep(0.3)# wait for a short time otherwise q_last is empty
    q_last=ur_reader.now_ur_pos
    x_last = vision_reader.pos
    k_last = vision_reader.k_pos
    k_last_h = hololens_reader.k_pos
    joint3pos_my = hololens_reader.joint3pos
    joint4pos_my = hololens_reader.joint4pos

    time.sleep(0.3)
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
        x = np.reshape(x,[2,1])
        k_now = vision_reader.k_pos
        actual_z = vision_reader.actual_z

        k_now_h = hololens_reader.k_pos
        d_slider = hololens_reader.pos
        d_slider = np.array(d_slider)

        joint3pos_my = []
        joint3pos_my = hololens_reader.joint3pos
        joint4pos_my = []
        joint4pos_my = hololens_reader.joint4pos

        J1,J2 = get_jacobian_for_hololens(urdf,q_now,0)
        # print('J1',J1)
        J1_3_dimension = J1[0:3,:]
        d1_3_dimension = np.dot(np.linalg.pinv(J1_3_dimension),joint3pos_my.reshape([3,1]))
        d1_3_dimension = np.asarray(d1_3_dimension).reshape(-1)
        d = np.concatenate((d1_3_dimension,np.array([0,0,0])),axis=0)
        # print("d1----------------------",d)

        J2_3_dimension = J2[0:3,:]
        d2_3_dimension = np.dot(np.linalg.pinv(J2_3_dimension),joint4pos_my.reshape([3,1]))
        d2_3_dimension = np.asarray(d2_3_dimension).reshape(-1)
        d = d + np.concatenate((d2_3_dimension,np.array([0,0])),axis=0) + d_slider

        # print('d======================', d)


        # 计算雅可比矩阵 J
        J,p = get_jacobian_from_joint(urdf,q_now,0)
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

            cbT = np.array([[-0.99884559, -0.04070438, -0.02550784,  0.02179647],
                            [ 0.02652412, -0.02462568, -0.99934481,  0.46575779],
                            [ 0.04004957, -0.99886772,  0.0256769,   1.38164686],
                            [ 0. ,         0.  ,        0.  ,        1.        ]])
            RR = cbT[0:3,0:3]

            # # theta_z = -np.ones([13,1])
            theta_z = [0.0,0.0,  0.0  ,0.0,0.0,0.0,   0.02*cbT[2,0]   ,   0.02*cbT[2,1]   ,   0.02*cbT[2,2]   ,cbT[2,0],cbT[2,1],cbT[2,2],cbT[2,3]]
            # theta_z[-4:] =[ 6.78457138e-01, -7.34604865e-01, -7.18357448e-03,  1.59173406e+00]
            theta_z[-4:] =[ 0.04004957, -0.99886772,  0.0256769,   0.98164686]
            theta_z = np.reshape(np.array(theta_z),[13,1])
            # theta_z =[ 0.04004957, -0.99886772,  0.0256769,   0.98164686]
            # theta_z = np.reshape(np.array(theta_z),[4,1])

            # RR = np.array([[-7.34639719e-01, -6.27919077e-04,  6.78457138e-01],\
            #                                 [-6.78432812e-01,  9.19848654e-03, -7.34604865e-01],\
            #                                 [-5.77950645e-03, -9.99957496e-01, -7.18357448e-03]])
            f = 2340
            u00 = 753
            v00 = 551
            theta_k0 = np.array([[f*RR[0,0]],[f*RR[0,1]],[f*RR[0,2]],[f*RR[1,0]],[f*RR[1,1]],[f*RR[1,2]],[RR[2,0]],[RR[2,1]],[RR[2,2]],\
                [u00*RR[2,0]],[u00*RR[2,1]],[u00*RR[2,2]],[v00*RR[2,0]],[v00*RR[2,1]],[v00*RR[2,2]]])
            theta_k = np.array([[0.0] for i in range(15)])
            fix_random = [1.1,0.8,0.85,1.2,0.85,1.0,1.05,1.05,1.15,1.2,1.16,0.98,0.95,0.90,1.19]
            for i in range(15):
                theta_k[i,0] = theta_k0[i,0]*fix_random[i]
            print('theta_z initial state:',theta_z)
            print('theta_k initial state:',theta_k)

        elif flag_initialized==2  and t-t_ready<2000:# 

            # 计算视觉矩阵Js
            u = x_now[0]-u0
            v = x_now[1]-v0
            z = actual_z # =========================
            Js = np.array([      [fx/z, 0, -u/z]     , [0, fy/z, -v/z]    ])
            Js = np.dot(Js,RR)
            # print(Js)
            Js_inv = np.linalg.pinv(Js)

            # 图像空间速度
            xdot = (x-x_last)/ros_freq
            # print('xdot',xdot)

            # 末端位置
            r4 = np.dot(p, [[0],[0],[0],[1]])
            r = r4[0:3]
            # print('r is:',r)

            # 末端速度
            rdot = np.dot(J , np.reshape(qdot,[6,1]))# rdot是个matrix!   np.dot的输出是matrix
            # print(type(rdot))

            # 计算深度自适应矩阵
            Y_z = np.array([[xdot[0,0]*p[0,0], xdot[0,0]*p[1,0], xdot[0,0]*p[2,0], xdot[0,0]*p[0,1], xdot[0,0]*p[1,1], xdot[0,0]*p[2,1], xdot[0,0]*p[0,2], xdot[0,0]*p[1,2], xdot[0,0]*p[2,2], xdot[0,0]*p[0,3], xdot[0,0]*p[1,3], xdot[0,0]*p[2,3], xdot[0,0]],\
                [xdot[1,0]*p[0,0], xdot[1,0]*p[1,0], xdot[1,0]*p[2,0], xdot[1,0]*p[0,1], xdot[1,0]*p[1,1], xdot[1,0]*p[2,1], xdot[1,0]*p[0,2], xdot[1,0]*p[1,2], xdot[1,0]*p[2,2], xdot[1,0]*p[0,3], xdot[1,0]*p[1,3], xdot[1,0]*p[2,3], xdot[1,0]]])
            # Y_z = np.array([[ xdot[0,0]*p[0,3], xdot[0,0]*p[1,3], xdot[0,0]*p[2,3], xdot[0,0]],\
            # [xdot[1,0]*p[0,3], xdot[1,0]*p[1,3], xdot[1,0]*p[2,3], xdot[1,0]]])
            # print('Yz',Y_z)
            
            # 计算相机参数自适应矩阵
            Y_k = np.array([[rdot[0,0], rdot[1,0], rdot[2,0],0,0,0, -x[0,0]*rdot[0,0], -x[0,0]*rdot[1,0], -x[0,0]*rdot[2,0], rdot[0,0], rdot[1,0], rdot[2,0], 0, 0, 0],\
                [0, 0, 0, rdot[0,0], rdot[1,0], rdot[2,0], -x[1,0]*rdot[0,0], -x[1,0]*rdot[1,0], -x[1,0]*rdot[2,0], 0, 0, 0,  rdot[0,0], rdot[1,0], rdot[2,0]]])
            # print('Yk',Y_k)

            # recover z_hat from theta_z
            z_hat = np.dot(np.array([p[0,0],  p[1,0],  p[2,0],  p[0,1],  p[1,1],  p[2,1],  p[0,2],  p[1,2],  p[2,2],  p[0,3],  p[1,3],  p[2,3], 1]), theta_z   )
            # z_hat = np.dot(np.array([p[0,3],  p[1,3],  p[2,3], 1]), theta_z[-4:]   )
            # print('p',p)    
            # print('z',z_hat)

            log_theta_z.append(list(theta_z))
            log_z_hat.append(list(z_hat))
            log_actual_z.append(actual_z)

            # recover Js_hat from theta_k
            Js_hat = np.array([[theta_k[0,0]-theta_k[6,0]*x[0,0]+theta_k[9,0], theta_k[1,0]-theta_k[7,0]*x[0,0]+theta_k[10,0], theta_k[2,0]-theta_k[8,0]*x[0,0]+theta_k[11,0]],\
                                                   [theta_k[3,0]-theta_k[6,0]*x[1,0]+theta_k[12,0], theta_k[4,0]-theta_k[7,0]*x[1,0]+theta_k[13,0], theta_k[5,0]-theta_k[8,0]*x[1,0]+theta_k[14,0]]])
            # print('Js_hat',Js_hat)
            Js_hat_inv = np.linalg.pinv(Js_hat)
            Js_ref = Js*actual_z
            # print('Js_ref',Js_ref)

            log_theta_k.append(list(theta_k))
            log_Js_hat.append(list(np.reshape(Js_hat,[-1,])))
            log_Js_ref.append(list(np.reshape(Js_ref,[-1])))
            

            # update
            theta_z_dot = -1/z_hat*np.dot(L_z, np.dot(Y_z.T, (x-dx)))/ros_freq
            # print('theta_z_dot',theta_z_dot)
            theta_z = theta_z+theta_z_dot
            theta_k_dot = 1/z_hat*np.dot(L_k, np.dot(Y_k.T, (x-dx)))/ros_freq
            theta_k = theta_k+theta_k_dot

            # 计算ut=================================
            # truth
            # ut = -np.dot( J_pos_inv, np.dot( Js_inv , np.dot(Kp, (x-dx) ) ) )
            # adaptive
            ut = -z_hat[0]*np.dot( J_pos_inv, np.dot( Js_hat_inv , np.dot(Kp, (x-dx) ) ) )

            # 计算un
            # if t-t_ready>15 and t-t_ready<16:
            #     # d = np.reshape(np.array([-0.2,-0.2,0.2,0.2,0.2,0.1],float),[6,1]  )
            #     # un = -np.dot(N_pos, np.dot(np.linalg.inv(Cd), d)  ) 
            #     # print(un)
            #     pass
            # else:
            #     un = np.zeros([6,1])

            d_array = np.reshape(d,[6,1])
            un = np.dot(N_pos, np.dot(np.linalg.inv(Cd), d_array ) ) 

            v = ut+un
            v = np.reshape(np.array(v),[-1,])

            # print('v',v.tolist())

            log_x.append(x.tolist())
            log_d.append(d.reshape([-1,]).tolist())
            log_q = np.concatenate((log_q,np.reshape(q_now,[6,1])),axis=1)
            log_qdot = np.concatenate((log_qdot,np.reshape(qdot,[6,1])),axis=1)
            log_dqdot = np.concatenate((log_dqdot,np.reshape(v,[6,1])),axis=1)
            log_rdot = np.concatenate((log_rdot,np.reshape(rdot,[6,1])),axis=1)

            # 保护
            v[v>0.4]=0.4
            v[v<-0.4]=-0.4

            urscript_speedj_pub(pub,v,a,0.1)
              
            # if k_now != k_last:
            #     urscript_speedj_pub(pub,v,a,0.1)
            # else:
            #     urscript_speedj_pub(pub,[0,0,0,0,0,0],a,0.1)

        q_last = q_now
        k_last = k_now
        k_last_h = k_now_h
        x_last = x
        t_last = t

        rate.sleep()# Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called. 这里就是ros_freq Hz
    # ===============================================while ends
    print(time.time()-t_start)

    # print(np.shape(log_qdot))
    log_x_array = np.array(log_x)
    log_theta_z_array = np.array(log_theta_z)
    log_theta_k_array = np.array(log_theta_k)
    log_z_hat_array = np.array(log_z_hat)
    log_d_array = np.array(log_d)
    log_actual_z_array = np.array(log_actual_z)

    np.save(current_path+ dir+'log_d.npy',log_d_array)
    np.save(current_path+ dir+'log_x.npy',log_x_array)
    np.save(current_path+ dir+'log_q.npy',log_q)
    np.save(current_path+dir+'log_qdot.npy',log_qdot)
    np.save(current_path+dir+'log_rdot.npy',log_rdot)
    np.save(current_path+dir+'log_dqdot.npy',log_dqdot)
    np.save(current_path+dir+'log_theta_z.npy',log_theta_z_array)
    np.save(current_path+dir+'log_theta_k.npy',log_theta_k_array)
    np.save(current_path+dir+'log_z_hat.npy',log_z_hat_array)
    np.save(current_path+dir+'log_Js_hat.npy',log_Js_hat)
    np.save(current_path+dir+'log_actual_z.npy',log_actual_z_array)

    # task space velocity==============================================
    plt.figure(figsize=(30,20))
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('Cartesian space velocity %d' % (j+1),fontsize=20)
        plt.xlabel('time (s)')
        if j<3:
            plt.ylabel('velocity (m/s)', fontsize = 18)
        else:
            plt.ylabel('angular velocity (rad/s)',fontsize = 18)

        plt.plot(np.linspace(0,np.shape(log_rdot)[1]/ros_freq,np.shape(log_rdot)[1]),np.reshape(np.array(log_rdot[j,:]),[-1,]) )
        # plt.legend()
    plt.savefig(current_path+ dir+'log_r.jpg')

    # vision space position===============================================
    plt.figure()
    plt.plot(log_x_array[:,0], log_x_array[:,1],label = 'actual')
    plt.scatter(dx[0,0],dx[1,0],label='target')
    plt.legend()
    plt.title('vision space trajectory')
    plt.xlabel('x (pixel)')
    plt.ylabel('y (pixel)')
    plt.savefig(current_path+ dir+'log_x.jpg')

    # vision space position verse time======================================
    fig = plt.figure(figsize=(20,8))
    plt.plot(np.linspace(0,np.shape(log_rdot)[1]/ros_freq,np.shape(log_rdot)[1]), log_x_array[:,0]-dx[0],label = 'x')
    plt.plot(np.linspace(0,np.shape(log_rdot)[1]/ros_freq,np.shape(log_rdot)[1]), log_x_array[:,1]-dx[1],label = 'y')
    plt.legend()
    # plt.title('vision space error')
    plt.xlabel('time (s)')
    plt.ylabel('error (pixel)')
    plt.savefig(current_path+ dir+'log_x_t.jpg',bbox_inches='tight',dpi=fig.dpi,pad_inches=0.0)

    # intention=========================================================
    plt.figure()
    for j in range(log_d_array.shape[1]):
        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),log_d_array[:,j],label = 'intention'+str(j+1))
    plt.legend()
    plt.xlabel('time (s)')
    plt.ylabel(' intention(rad/s)')
    plt.savefig(current_path+ dir+'log_d.jpg')

    # joint =================================================
    plt.figure(figsize=(30,20))
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('joint %d' % (j+1),fontsize=20)
        plt.xlabel('time (s)')
        plt.ylabel('angle (rad)')
        if j==4:
            # plt.ylim([1.2,1.5])
            pass
        elif j==5:
            plt.ylim([2.4,2.7])

        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),np.reshape(np.array(log_q[j,:]),[-1,]) )
    plt.savefig(current_path+ dir+'log_q.jpg')

    # joint space velocity=================================================
    plt.figure(figsize=(30,20))
    for j in range(6):
        ax = plt.subplot(3, 2, j+1)
        ax.set_title('joint space velocity %d' % (j+1),fontsize=20)
        plt.xlabel('time (s)')
        plt.ylabel('velocity (m/s)')

        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),np.reshape(np.array(log_qdot[j,:]),[-1,]) ,label='actual joint velocity')
        plt.plot(np.linspace(0,np.shape(log_dqdot)[1]/ros_freq,np.shape(log_dqdot)[1]),np.reshape(np.array(log_dqdot[j,:]),[-1,]) ,label='desired joint velocity')
        plt.legend()
    plt.savefig(current_path+ dir+'log_qdot.jpg')

    # theta_k========================================
    plt.figure()
    for j in range(15):
        lab = r'$\theta_k('+str(j+1)+')$'
        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),    log_theta_k_array[:,j]-log_theta_k_array[0,j],label = lab)
        plt.xlabel('time (s)')
        plt.title(r'elements of $\hat \theta_k$')
        plt.legend()
    plt.savefig(current_path+ dir+'log_thetak.jpg')

    # theta_z========================================
    plt.figure()
    for j in range(np.shape(log_theta_z_array)[1]):
        lab = r'$\theta_z('+str(j+1)+')$'
        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),    log_theta_z_array[:,j]-log_theta_z_array[0,j],label = lab)
        plt.xlabel('time (s)')
        plt.title(r'elements of $\hat \theta_z$')
        plt.legend()
    plt.savefig(current_path+ dir+'log_thetaz.jpg')

    # print(log_z_hat_array)
    # z_hat===========================================
    plt.figure()
    plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]), np.reshape(log_z_hat_array,[-1,]),label = 'z_hat')
    plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]), np.reshape(log_actual_z_array,[-1,]),label = 'z_actual')
    plt.legend()
    plt.title('z_hat')
    plt.xlabel('time (s)')
    plt.ylabel('depth (m)')
    plt.savefig(current_path+ dir+'z_hat.jpg')

    # Js_hat========================================
    
    log_Js_hat_array = np.array(log_Js_hat)
    log_Js_ref_array = np.array(log_Js_ref)
    for j in range(6):
        plt.figure()
        lab = r'$\hat{J}_s('+str(j+1)+')$'
        lab2 = r'$J_s('+str(j+1)+')$'
        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),    log_Js_hat_array[:,j],label = lab)
        plt.plot(np.linspace(0,np.shape(log_qdot)[1]/ros_freq,np.shape(log_qdot)[1]),    log_Js_ref_array[:,j],'--' ,label = lab2)
        plt.xlabel('time (s)')
        plt.title(r'elements of $\hat{J}_s$')
        plt.legend()
        plt.savefig(current_path+ dir+'log_Js_'+str(j+1)+'.jpg')

if __name__=="__main__":
    main()