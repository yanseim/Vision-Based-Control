#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy
from numpy import matlib,linalg
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
# from ur5_kinematics import Kinematic

def degree_to_rad(data):
    res=[]
    for i in range(len(data)):
        res.append(data[i]*numpy.pi/180)
    return res

def get_jacabian_from_joint():
    robot = URDF.from_xml_file("/home/roboticslab/ur_ws/src/tcst_pkg/urdf/ur5.urdf")
    # robot = URDF.from_xml_file(urdfname)
    tree = kdl_tree_from_urdf_model(robot)
    # print tree.getNrOfSegments()
    chain = tree.getChain("base_link", "tool0")
    # print chain.getNrOfJoints()
    
    # forwawrd kinematics
    kdl_kin = KDLKinematics(robot, "base_link", "tool0")
    # q=jointq
    q = [44.87,-70.67,40.06,299.46,-89.69,175.71]
    q=degree_to_rad(q)
    q=[2.4146897759547734, -1.1350234765509686, 0.5554777536842516, 5.258265530917523, -1.5663199722357237, 3.1563836819776188]
    pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
    print "pose-----",pose
    
    q_ik = kdl_kin.inverse(pose)  # inverse kinematics
    print "----------iverse-------------------\n", q_ik
    # print(pose)
    #print list(pose)
    # q0=Kinematic()
    # print(q0.Forward(q))
    # if flag==1:
    #     q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward(q))
    # else:
    #     q_ik = kdl_kin.inverse(pose)  # inverse kinematics
    # print "----------iverse-------------------\n", q_ik

    # if q_ik is not None:
    #     pose_sol = kdl_kin.forward(q_ik)  # should equal pose
    #     print "------------------forward ------------------\n",pose_sol
   
    J = kdl_kin.jacobian(q)
    print('kdl J:', J)
    return J,pose

if __name__ == "__main__":
    get_jacabian_from_joint()
