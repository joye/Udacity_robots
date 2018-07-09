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
[image9]: ./misc_images/urdf.png
[image10]: ./misc_images/base_to_end.png
[image11]: ./misc_images/DH_matrix.png
[image12]: ./misc_images/DH_Table_code.png
[image13]: ./misc_images/individual_trans.png
[image14]: ./misc_images/dh_tuidao.png
[image15]: ./misc_images/result.png
## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

From the picture I draw, I can derive the DH parameters as follows, the data is from kr210.urdf.xacro.
![alt text][image9]


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

Alpha_i-1 are angles between Z_i-1 and Z_i measured about X_i-1 in right-hand sense. From the picture, X_0 and X_1 are coincident, so alpha0 = 0; alpha1 = -pi/2 because turns of Z2 from Z1. Continuing with this logic and take the image in the above as information, can calculate the rest, alpha2 = 0, alpha3 = -pi/2, alpha4 = pi/2, alpha5 = -pi/2, alpha6 = 0.

A_i-1 is the distance from Z_i-1 to Z_i measured along X_i-1 where X_i-1 is perpendicular to both Z_i-1 and Z_i. 

Because Z0 and Z1 is in the same line, so A0 = 0.

A1's measurement can see from the top picture, where 0.35 is from kr210.urdf.xacro file.
```
<joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
```

A2's measurement use the same logic as A1's calculation.
```
<joint name="joint_3" type="revolute">
    <origin xyz="0 0 1.25" rpy="0 0 0"/>
```

A3's measurement:
```
  <joint name="joint_4" type="revolute">
    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
```

A_4 = 0 because Z4 and Z5 intersects in O4, so the distance is 0. The same as A_5.

A_6 = 0 because Z6 and Z_G is in the same line.

D_i is signed distance from X_i-1 to X_i measured along Z_i.

D1 can see from the above picture, where the values are from urdf file.
```
 <joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
```
```
 <joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
```
The other Di calculation is the similar logic as calcuating D1.

Theta_i is angle between X_i-1 to X_i measured about Z_i in right hand sense.  Theta_2 correponds of -pi/2 from X_1 to X_2 measured by Z_2. 

After calculating DH parameters, the individual transformation matrices can be seen in below.

![alt text][image11]
which is calculated through the below formula.
![alt text][image14]

Start with the base link and move link by link to the end effector, and use the DH parameters to build each individual transform.
![alt text][image10]
![alt text][image13]
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

The `IK_server.py` implementation use the method described in previous part. First create the DH parameter table,
![alt text][image12]
and then generate individual transformation matrices. 
for the loop, first get the end effector's position and orientation, from the calcuated euler angle, calculate the correct rotation matrix of end effector, then get the wrist center's position. After these parts, will calculate theta1 to theta6 using the equation I describe the pictures. When I check the result, can complete 8/10 pick and place cycles, one screenshot is in the below picture. Can see in the bin successfully finish 8/10 cycles.
![alt text][image15]
The [environment set up](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/af499fe6-fe20-4443-bb2f-f7d44d97b12f) is just the same as the link described. And I change demo mode to false in the inverse_kinematics.launch file under catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/launch/.
The project is launched by calling 
```sh
$ cd ~/catkin_ws/src/pick-place-robot/kuka_arm/scripts
$ ./safe_spawner.sh
```
and then in another terminal 
```sh
$ cd ~/catkin_ws/src/pick-place-robot/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```


