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
import numpy as np
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))

    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        ## Create symbols
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') 
	
	## Create Modified DH parameters
	s = {	alpha0: 0,	a0: 	0, 	d1:	0.75,	q1:q1,
		alpha1: -pi/2.,	a1:	0.35,	d2:	0,	q2:q2-pi/2.,
		alpha2: 0,	a2:	1.25,	d3:	0,	q3:q3,
		alpha3: -pi/2.,	a3:	-0.054,	d4:	1.5,	q4:q4,
		alpha4:	pi/2.,	a4:	0,	d5:	0,	q5:q5,
		alpha5: -pi/2.,	a5:	0,	d6:	0,	q6:q6,
		alpha6:	0,	a6:	0,	d7:	0.303,	q7:0
	}
		
	#
	## Define Modified DH Transformation matrix
	# Function for creating individual transformation matrices for each joint
	def TF_Matrix(alpha,a,d,q):
            TF = Matrix([
		[             cos(q),            -sin(q),            0,              a],
	       	[ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
	       	[ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
	       	[                   0,                   0,            0,               1]])
    	    return TF

	## Individual transformations for each join using the symbols ref. DH parameters
	T0_1  = TF_Matrix(alpha0,a0,d1,q1).subs(s)
    	T1_2  = TF_Matrix(alpha1,a1,d2,q2).subs(s)
    	T2_3  = TF_Matrix(alpha2,a2,d3,q3).subs(s)
    	T3_4  = TF_Matrix(alpha3,a3,d4,q4).subs(s)
   	T4_5  = TF_Matrix(alpha4,a4,d5,q5).subs(s)
    	T5_6  = TF_Matrix(alpha5,a5,d6,q6).subs(s)
    	T6_EE = TF_Matrix(alpha6,a6,d7,q7).subs(s)

	## Generalized homogeneous transform between base_link and gripper_link
    	T0_EE = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_EE
	
	## Creates rotation matrices
        r, p, y = symbols('r p y')
	R_x = Matrix([
		[ 1,              0,        0],
              	[ 0,        cos(r), -sin(r)],
              	[ 0,        sin(r),  cos(r)]])

	R_y = Matrix([
		[ cos(p),        0,  sin(p)],
              	[       0,        1,        0],
              	[-sin(p),        0,  cos(p)]])

	R_z = Matrix([
		[ cos(y), -sin(y),        0],
              	[ sin(y),  cos(y),        0],
              	[ 0,              0,        1]])

	## Extract rotation matrices from the transformation matrices
	# Difference between gripper reference frame as defined in the URDF vs. the DH params.
	# --> Intrinsic rotation 1) rotate 180 deg about Z axis, then 2) rotate -90 deg about Y axis
	# Correction:
	R_corr = R_z.subs(y,radians(180)) * R_y.subs(p,radians(-90))
	R_EE = R_z * R_y * R_x
	# Transform between base link and gripper link with correction
	REE_corr = R_EE*R_corr
        ###
	
	# Rotation Matrices from homogeneous transforms done in FK section
	# to build R0_3 
	R0_1 = T0_1[0:3,0:3]
	R1_2 = T1_2[0:3,0:3]	
	R2_3 = T2_3[0:3,0:3]
	
	# For use later in IK code inside for loop.
	R0_3 = R0_1*R1_2*R2_3
	
        # Initialize service response
	rospy.loginfo("Initialize service response")
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
	    	Rrpy = REE_corr.subs({'r':roll,'p':pitch,'y':yaw})
	    	R_EE = Rrpy
		# End effector position
	    	EE = Matrix([[px],[py],[pz]])
	    	#
	    	# Calculate joint angles using Geometric IK method
		# d7 = s[d7] = 0.303
	    	WC = EE -(0.303)*R_EE[:,2]
	        ####################################  
    		## theta1
    		####################################
	    	# Theta1 = atan2(wcy,wcx) (ref. L11,19)
	    	theta1 = atan2(WC[1],WC[0])
		#rospy.loginfo("Theta 1 is: %s"% theta1)

	    	# Theta 2 and theta 3 using diagram and cosine law
	    	# a1 = s[a1] = 0.35
	    	# a2 = s[a2] = 1.25
	    	# d1 = s[d1] = 0.75
	    	# a3 = s[a3] = -0.054
	    	# d4 = s[d4] = 1.5
	    	# Law of cosine Ref: https://en.wikipedia.org/wiki/Law_of_cosines
	    	sideA = sqrt(1.5*1.5+0.054*0.054)
		
	    	#xB = WC[0]-0.35
		#yB = WC[1]-0.75
		#sideB = sqrt(xB*xB+yB*yB)
		sideB = sqrt(pow((sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35),2)+pow((WC[2]-0.75),2))
	    	sideC = 1.25
	    	# Angles from cosine law formula
	    	angle_a = acos((sideB*sideB+sideC*sideC-sideA*sideA)/(2*sideB*sideC))
	    	angle_b = acos((sideA*sideA+sideC*sideC-sideB*sideB)/(2*sideA*sideC))
	    	angle_c = acos((sideB*sideB+sideA*sideA-sideC*sideC)/(2*sideA*sideB))
	    
	        ####################################  
	        ## theta 2 and theta 3
	        ####################################
	    	#theta2 = pi/2 - angle_a - atan2(yB,xB)
		#rospy.loginfo("Theta 2 is: %s"% theta2)
		# atan2(0.054,1.5) = 0.036 due to the -0.054 sag of link4
		theta2 = pi/2 -angle_a -atan2(WC[2]-0.75,sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
	    	theta3 = pi/2. - (angle_b + 0.036)
	    	#rospy.loginfo("Theta 3 is: %s"% theta3)
	    	# Inverse orientation
	    	# Equation for join 4,5,6
	    	R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
		# ortagoal matrices .transpose() = inv() -- transpose more stable. 
	    	R3_6 = R0_3.transpose()*R_EE	
		#rospy.loginfo("R3_6 is: %s"% R3_6)
	    	# Euler angles from rotation matrix

		####################################  
		## theta 4, theta 5 and theta 6
		#################################### 	    	
		# theta4,5,6 from R3_6
	    	# based on Lesson 11.8 Euler Angles from rotation matrix. 
	    	#theta4 = atan2(R3_6[2,2],-R3_6[0,2])
		#rospy.loginfo("Theta 4 is: %s"% theta4)
	    	#theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2]+R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
		#rospy.loginfo("Theta 5 is: %s"% theta5)
	    	#theta6 = atan2(-R3_6[1,1],R3_6[1,0])
		#rospy.loginfo("Theta 6 is: %s"% theta6)
		   
		### Theta 4-6 with tf.transformations.euler_from_matrix library
		# function takes a np.array type rotation matrix
		R3_6 = np.array(R3_6).astype(np.float64)
    
		theta4, theta5, theta6 = tf.transformations.euler_from_matrix(R3_6, axes = 'ryzy')
		
	    	# Populate response for the IK request
	    	# In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    	joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    	joint_trajectory_list.append(joint_trajectory_point)
		rospy.loginfo("Round %s in for loop"% x)
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
