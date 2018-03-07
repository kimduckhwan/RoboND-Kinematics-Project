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

[image1]: ./misc_images/DH.png
[image2]: ./misc_images/J1_J6.png
[image3]: ./misc_images/J1_J3.jpg
[image4]: ./misc_images/URDF.PNG

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This is the Writeup document

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.
#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

From the lecture, I used FK video 2 illustrating incremental change for each joint. 
![alt text][image4]

Based on the video and URDF file, I dervice DH parameters from kr210.urdf.xacro is below:
![alt text][image1]




#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

![alt text][image3]

Similar to the figure from the class, theta1 is easily calculated by using wx, wy since wx, wy are determined by only theta1.
For theta2 and theta3, length of three sides in triangle is calculated as 

l_j2_j3 (joint2 - joint3)      : 1.25 (a2 by definition)

l_wc_j3 (wrist_center - joint3): sqrt(1.5^2 + 0.054^2) since 1.5 distance by x direction, 0.054 distance by y direction

l_j2_wc (joint2 - wrist_center): sqrt(j2's height^2 + wc_distance_from_j1^2) = sqrt((wz-0.75)^2+(sqrt(wx^2+wy^2))^2-0.35)^2)
 

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

For detail about IK_server.py, please look at the code.

1. After implement IK_DEBUG.py, I copied them into IK_server.py, but since FK is not required in this project, I commented for T3_4,T4_5,T5_6,T6_G since I believe instancing those symbolic equation can increase memory overhead and delay

2. I tried to analyze runtime in two phases: theta1 ~ theta3, and theta4 ~ theta6. After pick up the blue stick, IK is computed to drop the object to the bin. Sometimes (not always, it may depend on the initial point - where the blue stick is located), computing theta4 ~ theta6 for moving to bin takes so long time (~50sec). I tried numpy.arctan2 instead of atan2 (I believe it's math.atan2), but can't get advantage. I also think about make a huge LUT for atan2 to save the compute latency, but not sure how fine grain is required. Still not sure why computing theta4~theta6 takes so long randomly.




