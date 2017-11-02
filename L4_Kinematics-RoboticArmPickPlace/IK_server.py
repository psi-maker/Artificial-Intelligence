#!/usr/bin/env python
# -*- coding: utf-8 -*
# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np
import matrix_create as m_init


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        
        #--below function is finished in file r0_3.m ------------
        # Create Modified DH parameters           
        # Define Modified DH Transformation matrix
        # perpare R0_3 for inverse kinematic 
        #------------------------------------------------------
        #R0_3 = pickle.load(open("r0_3.m", "rb"))        
        
        #get from r_rpy.m for R_rpy = Rot(Z,yaw−alpha)∗Rot(Y,pitch−beta)∗Rot(X,roll−gamma)∗R_corr        
        R_rpy = m_init.get_r_rpy()
        r, p, y = symbols('r p y')
        q1, q2, q3 = symbols('q1:4') # joint variables of theta
               
        # Extract rotation matrices from the transformation matrices
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
            R_rpy = R_rpy.subs({r: roll, p: pitch, y: yaw})          
            # Calculate joint angles using Geometric IK method, d_7 = s[d7] = 0.303; d_6 = s[d6] = 0
            wx = px - 0.303 * R_rpy[0,2]
            wy = py - 0.303 * R_rpy[1,2]
            wz = pz - 0.303 * R_rpy[2,2]
            
            #Joint 1, joint2, joint3
            theta1 = atan2(wy, wx)
            
            #cosbeta2 is calculated from another file            
            cosbeta2 = (0.4*(wz - 0.75)**2 + 0.4*(sqrt(wx**2 + wy**2) - 0.35)**2 - 0.2762004)/sqrt((wz - 0.75)**2 + (sqrt(wx**2 + wy**2) - 0.35)**2)
            #Set a limit over one to avoid imaginary numbers from the sqrt
            if cosbeta2 > 1:
                cosbeta2 = 1
            beta2 = atan2(sqrt(1-pow(cosbeta2,2)), cosbeta2) 
            #joint2 can have upper and lower pose to get the wrist center the same position, but because -45 < ($\theta2$) < 85, we can ignore the lower pose here
            beta1 = atan2(wz - 0.75, sqrt(wx**2 + wy**2) - 0.35)
            theta2 = np.pi/2 - beta1 - beta2
            
            #cos_angle_s4a2 = (pow(s4,2) + pow(1.25,2) - pow(s3,2)) / (2 * 1.25 * s4)
            cos_angle_s4a2 = -0.266489007328448*(wz - 0.75)**2 - 0.266489007328448*(sqrt(wx**2 + wy**2) - 0.35)**2 + 1.0167890739507           
            if cos_angle_s4a2 > 1:
                cos_angle_s4a2 = 1                
            angle_s4a2 = atan2(sqrt(1-pow(cos_angle_s4a2,2)), cos_angle_s4a2)
            theta3 = np.pi/2 - 0.036 - angle_s4a2           

            #get from r3_g.m             
            R3_G = m_init.get_r3_g()            
            R3_G = R3_G.subs({q1:theta1,q2:theta2,q3:theta3,r: roll, p: pitch, y: yaw})

            #compare the R3_G and R3_G_euler, we can get theta4,5,6            
            if R3_G[2,2] > 1:
                R3_G[2,2] = 1
        
            #joint5 can have left or right pose to get the gripper  the same postion, we need use python code to limit its value            
            theta5 = atan2(sqrt(R3_G[0,2]**2 + R3_G[2,2]**2), R3_G[1,2])
            if sin(theta5) < 0:
                theta4 = atan2(-R3_G[2,2],R3_G[0,2])
                theta6 = atan2(R3_G[1,1],-R3_G[1,0])                               
            else:
                theta4 = atan2(R3_G[2,2],-R3_G[0,2])
                theta6 = atan2(-R3_G[1,1],R3_G[1,0]) 
 
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()

