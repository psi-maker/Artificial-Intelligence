# -*- coding: utf-8 -*
import pickle
import os
import numpy as np
from mpmath import *
from sympy import *

curr_path = "/home/robond/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts/"
#print curr_path

# Define Modified DH Transformation matrix
def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[cos(q), -sin(q),        0,        a],
             [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
             [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha),  cos(alpha)*d],
             [0,           0,            0,        1]])
    return TF

def get_t03():
 # Create symbols
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


    

    # Create individual transformation matrices
    T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(s)
    T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(s)
    T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(s)
    #T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(s)
    #T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(s)
    #T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(s)
    #T6_G = TF_Matrix(alpha6, a6, d7, q7).subs(s)
    
    T0_3 = simplify(T0_1 * T1_2 * T2_3)
    #T0_6 = simplify(T0_1 * T0_2 * T0_3 * T3_4 * T4_5 * T5_6 * T6_G) # base link to link gripper
    
    return T0_3
  
def get_rpy_rcorr():
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
    R_rpy = R_rz * R_ry * R_rx
    return R_rpy, R_corr



if not os.path.exists(curr_path + "r_rpy.m"):    
    R_rpy = simplify(get_rpy_rcorr()[0] * get_rpy_rcorr()[1])
    pickle.dump(R_rpy, open(curr_path + "r_rpy.m", "wb"))
    print "First time to create r_rpy.m"
else:
    R_rpy = pickle.load(open(curr_path + "r_rpy.m", "rb"))
    print "R_rpy matrix is ready from file r_rpy.m"
    #print("R_rpy = : ", R_rpy.evalf())

#R3_G  =  (R0_3.inv("LU") * (R_rz * R_ry * R_rx)).subs({q1:theta1,q2:theta2,q3:theta3})
#changing the method of inverting R0_3 had an impact on the calculations. Maybe try using ADJ rather than LU,
#or even use the transpose due to its equivalency with the inverse for that type of matrix
if not os.path.exists(curr_path + "r3_g.m"):
    T0_3 = get_t03() 
    R0_3 = zeros(4,4)
    R0_3[:3, :3] = T0_3[:3, :3]
    R0_3[3, 3] = 1 # perpare for inverse kinematic      
    R3_G = simplify(R0_3.transpose() * get_rpy_rcorr()[0] * get_rpy_rcorr()[1] )
    pickle.dump(R3_G, open(curr_path + "r3_g.m", "wb"))
    print "First time to create r3_g.m"
else:
    R3_G = pickle.load(open(curr_path + "r3_g.m", "rb"))
    print "R3_G matrix is ready from file r3_g.m"
    #print("R3_G = : ", R3_G)
    

def get_r_rpy():
    return R_rpy

def get_r3_g():
    return R3_G

def calcu_angle():
    wx, wy, wz = symbols('wx wy wz')
    # from pic-2, DH table
    a_2 = 1.25
    d_4 = 1.50
    a_3 = 0.054
    a_1 = 0.35
    d_1 = 0.75
    s1 = sqrt(wx*wx+wy*wy) - 0.35
    s2 = wz - 0.75
    s3 = sqrt(pow(s1,2) + pow(s2,2))
    #s4 = sqrt(pow(0.054,2) + pow(1.50,2))
    s4 = 1.501
    beta1 = atan2(s2, s1)
    #beta4 = atan2(0.054, 1.50)
    #beta4 = 0.036
    cosbeta2 = simplify((pow(1.25,2) + pow(s3,2) - pow(s4,2)) / (2 * 1.25 * s3))
    #These formulas produce high round-off errors in floating point calculations if the triangle is very acute, i.e., 
    #if c is small relative to a and b or γ is small compared to 1. It is even possible to obtain a result slightly greater 
    #than one for the cosine of an angle.  #Cosine law: s4^2 = a2^2 + s3^2 - 2*as*s3*cos(beta2)

    cos_angle_s4a2 = simplify((2.253001 + 1.5625 - pow(s3,2)) / 3.7525)

    #print("cosbeta2 = : ",cosbeta2)
    #print("cos_angle_s4a2=: ",cos_angle_s4a2)
