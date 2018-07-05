#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
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

def rot_x(q):
    R = Matrix([[1, 0, 0],
               [0, cos(q), -sin(q)],
               [0, sin(q), cos(q)]])
    return R

def rot_y(q):
    R = Matrix([[cos(q), 0, sin(q)],
               [0, 1, 0],
               [-sin(q), 0, cos(q)]])
    return R


def rot_z(q):
    R = Matrix([[cos(q), -sin(q), 0],
               [sin(q), cos(q), 0],
               [0, 0, 1]])
    return R

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	#
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	#
	# Create Modified DH parameters
	#
        s = {alpha0 :     0, a0 :      0, d1 : 0.75, q1 : q1,
             alpha1 : -pi/2, a1 :   0.35, d2 :    0, q2 : q2 - pi/2,
             alpha2 :     0, a2 :   1.25, d3 :    0, q3 : q3,
             alpha3 : -pi/2, a3 : -0.054, d4 : 1.50, q4 : q4,
             alpha4 :  pi/2, a4 :      0, d5 :    0, q5 : q5,
             alpha5 : -pi/2, a5 :      0, d6 :    0, q6 : q6,
             alpha6 :     0, a6 :      0, d7 : 0.303, q7 : 0} 

	#
	# Define Modified DH Transformation matrix
	#
        def get_TF(alpha, a, d, q):
            Tf = Matrix([[cos(q), -sin(q), 0, a],
                         [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                         [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
                         [0,0,0,1]])
            return Tf
        
        #
	# Create individual transformation matrices
	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	#
        ###
        T0_1 = get_TF(alpha0, a0, d1, q1).subs(s)
        T1_2 = get_TF(alpha1, a1, d2, q2).subs(s)
        T2_3 = get_TF(alpha2, a2, d3, q3).subs(s)
        T3_4 = get_TF(alpha3, a3, d4, q4).subs(s)
        T4_5 = get_TF(alpha4, a4, d5, q5).subs(s)
        T5_6 = get_TF(alpha5, a5, d6, q6).subs(s)
        T6_G = get_TF(alpha6, a6, d7, q7).subs(s)
        T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)
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
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
            #R_z = Matrix([ [cos(np.pi), -sin(np.pi), 0],
            #               [sin(np.pi), cos(np.pi),  0],
            #               [0,                   0,  1]])
            #R_y = Matrix([[cos(-np.pi/2),  0, sin(-np.pi/2)],
            #              [0,  1, 0],
            #              [-sin(-np.pi/2), 0, cos(-np.pi/2)]])
            #R_corr = simplify(R_z * R_y)
            R_corr = rot_z(np.pi) * rot_y(-np.pi/2)

            r, p, y = symbols('r p y')
            Rot_x = rot_x(r)
            Rot_y = rot_y(p)
            Rot_z = rot_z(y)

            ROT_G = Rot_z * Rot_y * Rot_x

            ROT_G = ROT_G * R_corr
            ROT_G = ROT_G.subs({'r' : roll, 'p' : pitch, 'y' : yaw})
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###
            G = Matrix([[px], [py], [pz]])

            WC = G - 0.303 * ROT_G[:, 2]

            theta1 = atan2(WC[1], WC[0])

            a_side = sqrt(pow(1.5, 2) + pow(0.054, 2))
            c_side = 1.25
            b_side = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))

            anglea = acos((b_side * b_side + c_side * c_side - a_side * a_side) / (2 * b_side * c_side))
            angleb = acos((a_side * a_side + c_side * c_side - b_side * b_side) / (2 * a_side * c_side))
            anglec = acos((b_side * b_side + a_side * a_side - c_side * c_side) / (2 * b_side * a_side))

            theta2 = pi/2 - anglea - atan2(WC[2]-0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)

            theta3 = pi/2 - angleb - atan2(0.054, 1.5)

            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]

            R0_3 = R0_3.evalf(subs = {q1 : theta1, q2 : theta2, q3 : theta3})

            R3_6 = R0_3.inv("LU") * ROT_G

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2]+R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            # Populate response for the IK request
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
