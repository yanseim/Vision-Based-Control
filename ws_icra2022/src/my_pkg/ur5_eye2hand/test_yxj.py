#encoding: utf-8


import numpy as np
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import os

def get_jacobian_for_hololens(urdfname,jointq,flag):
    robot = URDF.from_xml_file(urdfname)
    kdl_kin = KDLKinematics(robot, "base_link", "forearm_link")
    q=jointq
    #q = [0, 0, 1, 0, 1, 0]
    pose = kdl_kin.forward(q[:3])  # forward kinematics (returns homogeneous 4x4 matrix)
    pose[0,:],pose[1,:] = -pose[0,:],-pose[1,:]# added on 0728 yxj
    # 注意这里是从base_link到wrist_3_link。调外参的时候是base到wrist_3_link，相差z轴转180度。这里转了之后就相当于从base开始了 0826 yxj

    J = kdl_kin.jacobian(q)
    J[0,:],J[1,:],J[3,:],J[4,:]  = -J[0,:],-J[1,:],-J[3,:],-J[4,:] # added on 0728 yxj
    #print 'J:', J
    return J,pose

if __name__=="__main__":
    urdf =  "/home/robotics/Robotics/Research/yxj/ws_icra2022/src/my_pkg/urdf/ur5.urdf"
    joint3pos = [1,0,0]
    joint3pos.extend([0,0,0])
    print(joint3pos)
    v = joint3pos/np.linalg.norm(joint3pos)
    J1,p1 = get_jacobian_for_hololens(urdf,[0.1,0,0,0,0,0],0)
    d_3_dimension = np.dot(np.linalg.pinv(J1),v)
    d_3_dimension = np.asarray(d_3_dimension).reshape(-1)
    print(d_3_dimension)
    d = np.concatenate((d_3_dimension,np.array([0,0,0])),axis=0)
    print(d)