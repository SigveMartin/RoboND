## Project: Kinematics Pick & Place
### Project 2 in Robotics Engineer Nanodegree from Udacity. 

---


**Steps to complete the project:** 


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: /images/robot_schematic.jpg
[image2]: /images/dh_vs_urdf.jpg
[image3]: /images/dh_table.jpg
[image4]: /images/PandP1.png
[image5]: /images/PandP2.png
[image6]: /images/dh-transform.png
[image7]: /images/dh-transform-matrix.png
[image8]: /images/eq3.png
[image9]: /images/homo_xform-2.png
[image10]: /images/wc_equations.png
[image11]: /images/l21-l-inverse-kinematics-new-design-fixed.jpg
[image12]: /images/schematic_calc_thetas.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

As recomended in the project rubrics I schetched out a schematic view of the robot and anotated all joints and links as descibed in image 1 below: 

![Schematic view of K210 for constructing the DH parameter table][image1]

To derive the aplhas and to compare the kr210.urdf.xacro file I draw up the schematic view according to DH, and also the schematic view according to the urdf file. The frames are different, which needs to be accounted for when building the DH parameter table as shown in image 2 below: 

![Schematic vies of kr210 according to DH to the left, and URDF to the right.][image2]

This lead to the following DH parameter table definition, as shown in image 3 below and table 1.

![DH parameter table setup][image3]

### Table 1 DH Parameters

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1:q1
1->2 | -pi/2 | 0.35 | 0 | q2:q2-pi/2
2->3 | 0 | 1.25 | 0 | q3:q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4:q4
4->5 | pi/2 | 0 | 0 | q5:q5
5->6 | -pi/2 | 0 | 0 | q6:q6
6->EE | 0 | 0 | 0.303 | q7:0


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Individual transform matrices about each joint (note: in IK_server.py I have captured this in a function, to make the script simpler). 

Following the DH convention for individual transforms beteen link i-1 and i:

![DH convention for individual transforms formula (ref. Project Robotic Arm: Pick & Place, 13)][image6]

In matrix form this transform is: 

![DH convention for individual transforms matrix form (ref. Project Robotic Arm: Pick & Place, 13)][image7]


T0_1 = Matrix([
	[             cos(q1),            -sin(q1),            0,              a0],
       	[ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
       	[ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
       	[                   0,                   0,            0,               1]])
T1_2 = Matrix([
	[             cos(q2),            -sin(q2),            0,              a1],
       	[ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
       	[ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
       	[                   0,                   0,            0,               1]])
T2_3 = Matrix([
	[             cos(q3),            -sin(q3),            0,              a2],
       	[ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
       	[ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
       	[                   0,                   0,            0,               1]])
T2_3 = Matrix([
	[             cos(q3),            -sin(q3),            0,              a2],
       	[ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
       	[ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
       	[                   0,                   0,            0,               1]])
T3_4 = Matrix([
	[             cos(q4),            -sin(q4),            0,              a3],
       	[ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
       	[ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
       	[                   0,                   0,            0,               1]])
T4_5 = Matrix([
	[             cos(q5),            -sin(q5),            0,              a4],
       	[ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
       	[ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
       	[                   0,                   0,            0,               1]])
T5_6 = Matrix([
	[             cos(q6),            -sin(q6),            0,              a5],
       	[ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
       	[ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
       	[                   0,                   0,            0,               1]])
T6_EE = Matrix([
	[             cos(q7),            -sin(q7),            0,              a6],
       	[ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
       	[ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
       	[                   0,                   0,            0,               1]])

Generalized homogeneous transform between base_link and gripper_link:

T0_EE = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_EE

This follows the following matrix structure: 

![Matrix structure of transformation matrix T0_EE (ref. Project Robotic Arm: Pick & Place, 13)][image8]

consisting of both a rotation part (RT) and a position part (Px,Py,Pz). 

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

In order to find theta 1, 2 and 3 we consider the Inverse Position Kinematics. We use the position coordinates of the end effector that we get from the simulator, and use the following equations to get the position of the Wrist Center (WC) (joint 4,5,6 ref. DH). 

The transform T0_EE or Rrpy could be described like the following matrix: 

![Matrix structure of transformation matrix T0_EE (ref. Project Robotic Arm: Pick & Place, 15)][image9]

where Px,Py, Pz is end-effector positions and l, m, n are ortonormal vectors representing the EE orientation along X,Y, Z of the local coordinate frame. Since n is the vector along z-axis of the gripper_link the followingf equation is valid: 

![Equations to get positon of wrist center (ref. Project Robotic Arm: Pick & Place, 15)][image10]

Px,Py,Pz is position of EE and Wx,Wy,Wz is wrist center position relative to world/base coordinate frame. We can substitute D6+l with D7 from our DH table, as this is the length between WC and EE. 

We get nx,ny and nz from Rrpy. In order to get Rrpy we use the following equation: 

Rrpy = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll) * R_corr

where R_corr is a coorection to compensate for the difference between gripper reference frame as defined in URDF vs. our DH parameters. It is two intrinsic rotations, first 180 degrees about Z and then -90 degrees about Y. Then we have nx, ny and nz to get WC. 

We also have that Rrpy equals the rotation matrix from base to end effector (R_EE) so:

R_EE = R_z * R_y * R_x
R_err = R_z.subs(y,radians(180)) * R_y.subs(p,radians(-90))
R_EE = R_EE * R_err
R_EE = R_EE.subs({'r':roll,'p':pitch,'y':yaw}) -- got roll, pitch, yaw from simulator

EE = Matrix([[px],[py],[pz]]) -- got px,py,pz from simulator
    
WC = EE - (d7)*R_EE[:,2]

When we have this position we could get theta 1, through formula for getting Euler angles from rotation matrices:

Theta 1 = atan2(WCy,WCx) [ref. lesson 11 page 19](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/8d553d46-d5f3-4f71-9783-427d4dbffa3a).

We got the following visualization provided of theta 2 and theta 3: 

![Visualization of theta 2 and theta 3 (ref. Project Robotic Arm: Pick & Place, 15)][image11]

In addition I drew up the following diagram and placed known lengths onto in, also encompasing theta1. 

![Visualization of theta 1, theta 2 and theta 3][image12]

This is a three dimentional visualization where (1) is joint 1, (2) is joint 2 and (3) is joint 3. x1,y1,z1 describes base frame. and x2,y2,z2 describes frame of joint 2. Important to note that WC is given according to x1,x2,x3. 

Here we use the [law of cosines](https://en.wikipedia.org/wiki/Law_of_cosines) to get theta 2 and theta 3. 

Side C is taken directly from the DH parameters as a2 = 1.25. In order to find side A we need d4 and a3 from the DH table. 

Side A = sqrt(d4²+a3²)

In order to find side B we need to use that we know the vector from base to WC. We also need to find the vector from 1 to 2, wich we have called L. 

Side B = vectorWC-L = sqrt((sqrt(Px²+Py²)-a1)²+(Pz-d1)²)

side C = a2 

This way we have all sides of the triangle 2-3-WC, and can use law of cosine to get angles a, b and c as described in image 12. 
a = acos((B * B + C * C - A * A)/(2*B*C))
b = acos((A*A+C*C-B*B)/(2*A*C))
c = acos((B*B+A*A-C*C)/(2*A*B))

As seen from image 12 theta 2 = 90 - a - alpha. Where alpha is the angle between side B and x2. Ergo:

Theta 2 = pi/2 - a - atan2(Pz-d1,sqrt(Px²+Py²)-a1)

Theta 3 is as seen from image 12 90 degrees minus b and the angle between d4 and A. Thus, 

Theta 3 = pi/2 - b - atan2(a3,d4)

Then we have found theta 1, 2 and 3 through Inverse Position Kinematics, and could go on and use Inverse Orientation Kinematics to derive theta 4,5 and 6. 

We already know that Rrpy = R_EE, that is the Rotation between base link and gripper link. This needs to be equal to the products of individual rotations between respective links: 

R_EE = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6*R6_EE = Rrpy

We could find R0_3 = R0_1*R1_2*R2_3 and use our newly found theta 1, 2 and 3 in this matrix. We have Rrpy = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll) * R_corr. And we then could get R3_6, which is the rotation matrix between link 3 and the end effector, by:

R3_6 = inv(R0_3) * Rrpy
We can get R3_6 in its symbolic form by, R3_6 = R3_4*R4_5*R5_6
R3_6 = Matrix([
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])

We then extract theta 4 (q4), theta 5 (q5) and theta 6 (q6) as Euler angles from this rotation matrix (as described in [Lesson 11 page 8](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/a124f98b-1ed5-45f5-b8eb-6c40958c1a6b)). 

In this project we used the functions from tf.transformations.euler_from_matrix by feeding it the R3_6 rotation matrix. 

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The gode is provided in IK_server.py. I had some difficulties especially in getting the theta 4 to 6. I ended up using the tf.transformations.euler_from_matrix function. I tested and debugged a lot with different orientations as parameters, but Rotation seqence ryzy seemed to have the least amount of error, and also was able to successfully perform pick and place operations. Also tried manually getting the euler angles, however since the matrix is not equal to the example in the lectures I had some difficulties getting this. I will revisit this, and then look into this part in more detail getting it right. 

I implemented most of the performance improvements, eg. directly using values instad of eg. DH params instead of getting them from the DH dictionary each time. After a while I felt the code got to a ok performance level. 

Here is a video showing a round of pick and place in Gazebo: 

[![KR210 in Gazebo pick and place demo](https://img.youtube.com/vi/OVSftKSRyO0/0.jpg)](https://www.youtube.com/watch?v=OVSftKSRyO0)


