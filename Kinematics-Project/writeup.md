## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/dh_pic.jpg
[image5]: ./misc_images/homogenous.jpg
[image6]: ./misc_images/theta1_2.jpg
[image7]: ./misc_images/theta3.jpg
[image8]: ./misc_images/theta4_6.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

From the picture I draw, I can derive the DH parameters as follows:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.33+0.42 = 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 0.96+0.54 = 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.193+0.11 = 0.303 | 0

![alt text][image4]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
When derive the DH parameter, the coordinate system is not the same as reference frame, so need to do some transformation as the below picture works. 
![alt text][image5]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Use joint 5 as wrist center, first need to calculate joint 5's position in local coordinate frame. After calcuate wrist center's position, calculate each joint angles. The calculation for theta1 theta2, theta3 are as below pics.
![alt text][image6]
![alt text][image7]

After calculate theta1 to theta3, use the below pic to calculate theta4, theta5 and theta6.
![alt text][image8]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The `IK_server.py` implementation use the method described in previous part. First create the DH parameter table, and then generate individual transformation matrices. for the loop, first get the end effector's position and orientation, from the calcuated euler angle, calculate the correct rotation matrix of end effector, then get the wrist center's position. After these parts, will calculate theta1 to theta6 using the equation I describe the pictures. When I check the result, can complete 8/10 pick and place cycles, attach one video screenshot.

<p align="center">
<iframe height=498 width=510 src="./misc_images/one_catch.mp4">
</p>
