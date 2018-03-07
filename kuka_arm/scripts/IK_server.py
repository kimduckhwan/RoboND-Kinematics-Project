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
import timeit
# Common elements

dtr = pi/180
rtd = 180/pi

qx, qy, qz = symbols('qx qy qz')
R_x = Matrix([[1, 0,      0       ],
              [0, cos(qx), -sin(qx) ],
              [0, sin(qx), cos(qx) ]])
R_y = Matrix([[cos(qy),  0, sin(qy)],
              [0,       1,      0],
              [-sin(qy), 0, cos(qy)]])
R_z = Matrix([[cos(qz), -sin(qz), 0],
              [sin(qz), cos(qz),  0],
              [0,      0,       1]])







def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols

        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #  theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0,alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Based on KUKA KR210
	# Create Modified DH parameters
	DH = {alpha0: 0,        a0: 0,      d1: 0.75,       q1: q1,
              alpha1: -pi/2,    a1: 0.35,   d2: 0,          q2: q2-pi/2,
              alpha2: 0,        a2: 1.25,   d3: 0,          q3: q3,
              alpha3: -pi/2,    a3: -0.054, d4: 1.50,       q4: q4,
              alpha4: pi/2,     a4: 0,      d5: 0,          q5: q5,
              alpha5: -pi/2,    a5: 0,      d6: 0,          q6: q6,
              alpha6: 0,        a6: 0,      d7: 0.303,      q7: 0}

        # Define Modified DH Transformation matrix
	#
	#
        # Create individual transformation matrices
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                       [                   0,                   0,            0,               1]])

        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [                   0,                   0,            0,               1]])

        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                        [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                        [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                        [                   0,                   0,            0,               1]])

        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                        [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                        [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                        [                   0,                   0,            0,               1]])

        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                        [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                        [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                        [                   0,                   0,            0,               1]])

        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                        [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                        [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                        [                   0,                   0,            0,               1]])

        T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                        [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                        [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                        [                   0,                   0,            0,               1]])

        T0_1 = T0_1.subs(DH)
        T1_2 = T1_2.subs(DH)
        T2_3 = T2_3.subs(DH)
#        T3_4 = T3_4.subs(DH)
#        T4_5 = T4_5.subs(DH)
#        T5_6 = T5_6.subs(DH)
#        T6_G = T6_G.subs(DH)
#        T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)

        # Correction needed to account of orientation between URDF - DH
        # Intrinsic z(180deg) * y (-90deg)
        #z(180deg)



        R_correction = simplify(R_z * R_y)
        R_correction = R_correction.subs({'qz':pi, 'qy':-pi/2})


        R_rpy = simplify(R_z * R_y * R_x) # 3 x 3
        R_rpy = R_rpy * R_correction

        #T_rpy = R_rpy.row_join(Matrix([[0],[0],[0]])).col_join(Matrix([[0,0,0,1]])) # 4 x 4
        #T_rpy = T_rpy * R_correction

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):

            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # first, the Cartesian coordinates of the wrist center, and then the composition of rotations to orient the end effector
	    # first three joints to control the position of the wrist center
            # while the last three joints would orient the end effector as needed.

            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z


            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            print('[%d] poses: %f %f %f, %f %f %f' %(x,px,py,pz,roll,pitch,yaw))
            start = timeit.default_timer()
            #T_rpy = T_rpy.evalf(subs={qx:roll, qy:pitch, qz:yaw})
            R_rpy = R_rpy.evalf(subs={qx:roll, qy:pitch, qz:yaw})

            # Wrist center's position
            d7 = 0.303
            wx = px - d7 * R_rpy[0,2]
            wy = py - d7 * R_rpy[1,2]
            wz = pz - d7 * R_rpy[2,2]

            # Find q1 (theta1), q2, q3 such that WC has coordinates equal to equation (3)

            theta1 = atan2(wy,wx)
            r = sqrt(wx*wx + wy*wy)

            # follow fig notation
            # l_wc_j3*l_wc_j3 = l_j2_j3*l_j2_j3 + l_j2_wc*l_j2_wc - 2*l_j2_wc*l_j2_j3*cos(alpha)
            # l_wc_j3*l_wc_j3 - l_j2_j3*l_j2_j3 - l_j2_wc*l_j2_wc = -2*l_j2_wc*l_j2_j3*cos(alpha)
            # cos(alpha) = l_wc_j3*l_wc_j3 - l_j2_j3*l_j2_j3 - l_j2_wc*l_j2_wc/-2*l_j2_wc*l_j2_j3
            l_wc_j3 = sqrt(1.50*1.50 + 0.054*0.054)
            l_j2_wc = sqrt((r-0.35)*(r-0.35)+(wz-0.75)*(wz-0.75))
            l_j2_j3 = 1.25

            A = l_wc_j3
            B = l_j2_wc
            C = l_j2_j3

            a = acos ((B*B+C*C-A*A)/(2*B*C))
            theta2 = pi/2 - a - atan2(wz-0.75, r-0.35)

            b = acos ((A*A+C*C-B*B)/(2*A*C))
            theta3 = pi/2 - (b+0.036)       # 0.036 = asin(0.054/1.5)
            stop1 = timeit.default_timer()
            print("theta1 ~ 3 are done")
            print(stop1-start)

            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
            R3_6 = R0_3.inv("LU")*R_rpy
                    #inv(R0_3)*R_rpy



            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            print("[theta4] %f %f %f" %(R3_6[2,2], -R3_6[0,2], theta4))

            a = sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2])
            theta5 = atan2(a, R3_6[1,2])
            print("[theta5] %f %f %f" %(a, R3_6[1,2], theta5))

            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            print("[theta6] %f %f %f" %(-R3_6[1,1], R3_6[1,0], theta6))


            print("theta4 ~ 6 are done")
            stop2 = timeit.default_timer()
            print(stop2-stop1)


	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)
            end = timeit.default_timer()
            print(end-stop2)

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
