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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Create symbols
        #print("BEGIN INVERSE KINEMATICS CALCULATION")
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # joint variables of theta 
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offset d_i = signed distance from X_i-1 to X_i along Z_i
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # non-zero link length a_i-1 = Z_i-1 to Z_i along X_i-1 where X_i-1 is perpendicular to both Z_i-1 to Z_i 
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angel alpha_i-1 = Z_i-1 and Z_i measured about X_i-1 in a right-hand sense  
   
        # Create Modified DH parameters
        s = {alpha0: 0,     a0: 0,      d1: 0.75, 
             alpha1: -pi/2, a1: 0.35,   d2: 0,     q2: q2-pi/2,  
             alpha2: 0,     a2: 1.25,   d3: 0,
             alpha3: -pi/2, a3: -0.054, d4: 1.50,
             alpha4: pi/2,  a4: 0,      d5: 0,
             alpha5: -pi/2, a5: 0,      d6: 0,
             alpha6: 0,     a6: 0,      d7: 0.303, q7: 0}     
        
        # Define Modified DH Transformation matrix
        def TF_Matrix(alpha, a, d, q):
            TF = Matrix([[cos(q),        -sin(q),        0,        a],
                     [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                     [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha),  cos(alpha)*d],
                     [0,           0,            0,        1]])
            return TF
    
        # Create individual transformation matrices
        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(s)
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(s)
        T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(s)
        T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(s)
        T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(s)
        T6_G = TF_Matrix(alpha6, a6, d7, q7).subs(s)
        
        T0_3 = T0_1 * T1_2 * T2_3 
        R0_3 = zeros(4,4)
        R0_3[:3, :3] = T0_3[:3, :3]
        R0_3[3, 3] = 1 # perpare for inverse kinematic
        
        T0_G = T0_3 * T3_4 * T4_5 * T5_6 * T6_G # base link to link gripper
 
        # Euler angle calculation
        # Define RPY rotation matrices http://planning.cs.uiuc.edu/node102.html
        #Rot(Z,yaw−alpha)∗Rot(Y,pitch−beta)∗Rot(X,roll−gamma)∗R_corr
        r, p, y = symbols('r p y')
        R_rx = Matrix([[ 1,              0,        0, 0],
                          [ 0,        cos(r), -sin(r), 0],
                          [ 0,        sin(r),  cos(r), 0],
                          [0, 0, 0, 1]])
        R_ry = Matrix([[ cos(p),        0,  sin(p), 0],
                          [       0,        1,        0, 0],
                          [-sin(p),        0,  cos(p), 0],
                          [0, 0, 0, 1]])
        R_rz = Matrix([[ cos(y), -sin(y),        0, 0],
                          [ sin(y),  cos(y),        0, 0],
                          [ 0,              0,        1, 0],
                          [0, 0, 0, 1]])
        R_corr = R_rz.subs(y, np.pi) * R_ry.subs(p, -np.pi/2) # The correction for DH coordinate and URDF coordinate
        R_rpy = R_rz * R_ry * R_rx * R_corr

        
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
            # from pic-2, DH table
            a_2 = s[a2] #= 1.25
            d_4 = s[d4] #= 1.50
            a_3 = abs(s[a3]) #= 0.054
            a_1 = s[a1] #= 0.35
            d_1 = s[d1] #= 0.75
            s1 = sqrt(wx*wx+wy*wy) - 0.35
            s2 = wz - 0.75
            s3 = sqrt(pow(s1,2) + pow(s2,2))
            #s4 = sqrt(pow(0.054,2) + pow(1.50,2))
            s4 = 1.501
            beta1 = atan2(s2, s1)
            #beta4 = atan2(0.054, 1.50)
            #beta4 = 0.036
            cosbeta2 = (pow(1.25,2) + pow(s3,2) - pow(s4,2)) / (2 * 1.25 * s3)
            #These formulas produce high round-off errors in floating point calculations if the triangle is very acute, i.e., 
            #if c is small relative to a and b or γ is small compared to 1. It is even possible to obtain a result slightly greater 
            #than one for the cosine of an angle.  #Cosine law: s4^2 = a2^2 + s3^2 - 2*as*s3*cos(beta2)
            #Set a limit over one to avoid imaginary numbers from the sqrt
            #if cosbeta2 > 1:
            #    cosbeta2 = 1
            if cosbeta2 > 1:
                cosbeta2 = 1
            beta2 = atan2(sqrt(1-pow(cosbeta2,2)), cosbeta2)
            #beta2 = acos((pow(1.25,2) + pow(s3,2) - pow(s4,2)) / (2 * 1.25 * s3))            
            theta2 = np.pi/2 - beta1 - beta2
            
            cos_angle_s4a2 = (pow(s4,2) + pow(1.25,2) - pow(s3,2)) / (2 * 1.25 * s4)
            if cos_angle_s4a2 > 1:
                cos_angle_s4a2 = 1                
            #angle_s4a2 = acos((pow(s4,2) + pow(1.25,2)-pow(s3,2))/(2 * 1.25 * s4))
            angle_s4a2 = atan2(sqrt(1-pow(cos_angle_s4a2,2)), cos_angle_s4a2)
            theta3 = np.pi/2 - 0.036 - angle_s4a2           

            #R3_G  =  (R0_3.inv("LU") * (R_rz * R_ry * R_rx)).subs({q1:theta1,q2:theta2,q3:theta3})
            # changing the method of inverting R0_3 had an impact on the calculations. Maybe try using ADJ rather than LU, 
            #or even use the transpose due to its equivalency with the inverse for that type of matrix
            R0_3 = R0_3.subs({q1:theta1,q2:theta2,q3:theta3})           
            R3_G  =  (R0_3.transpose()) * R_rpy

            #compare the R3_G and R3_G_euler, we can get theta4,5,6
            #theta4 = atan2(R3_G[2,2],R3_G[0,2])
            #theta5 = atan2(sqrt(R3_G[0,2] * R3_G[0,2] + R3_G[2,2] * R3_G[2,2]), R3_G[1,2])
            #theta6 = atan2(-R3_G[1,1], R3_G[1,0])
            if R3_G[2,2] > 1:
                R3_G[2,2] = 1
            #compare the R3_G and R3_G_euler, we can get theta4,5,6
            theta4 = atan2(R3_G[1,2],R3_G[0,2])
            theta5 = atan2(sqrt(1 - pow(R3_G[2,2],2)), R3_G[2,2]) - np.pi
            theta6 = atan2(-R3_G[2,1], R3_G[2,0])
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

