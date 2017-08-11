[//]: # (Image References)
[image_multi_process_structure]: ./misc/multi_process_structure.png
[image_multi_function_chart]: ./misc/multi_function_chart.JPG
[image_decision_making]: ./misc/decision_making.png
[image_gripper]: ./misc/gripper.JPG
[image_gripper2]: ./misc/gripper2.JPG
[image_DH_table]: ./misc/DH_table.png
[image_3modes]: ./misc/3modes.PNG
[image_hardware]: ./misc/hardware.png
[image_hardware2]: ./misc/hardware2.png
[image_performance]: ./misc/performance.PNG


# Multi-process, Multi-threaded System for 6 DOF Robotics RGBD Vision Arm

The Goal of this project is to design a 6 DOF arm and to develop algorithms for effective task completion under time constraints as well as navigation requirement to determine best possible path. This is achieved by initially constructing a 3D location via Kinect RGBD camera that records synchronized and aligned RGB, depth images. Blob detection computer vision technique is used to locate different blocks. We design three different inverse kinematics modes to make the most of the gripper in order to augment the robot’s workspace to meet different task requirement.


## Task

**Event 1**: Pick n’ place (100 points)
##### 3 Blocks will be placed on the +y of the board (not stacked) with >2.5cm of space in between
##### Move the 3 blocks to the other side of the board, at the same location but in -y
##### 15 points for each block picked
##### 15 points for each block placed
##### 2.5 points for each 10s less than 40s it takes to complete the task (up to 10 points)

**Event 2**: Pick n’ stack (100 points)
##### 3 Blocks will be placed on the +y of the board (not stacked) with >2.5cm of space in between
##### Move the 3 blocks to the other side of the board and stack them at any location
##### 15 points for each block picked
##### 15 points for each block stacked
##### 2.5 points for each 10s less than 40s it takes to complete the task (up to 10 points)

**Event 3**: Line ‘em up! (150 points)
##### Blocks will be placed in a specific configuration, some will be stacked (no more than 3 high)
##### Order the Blocks in a line according to the resistor color code: 
##### Black Red Orange Yellow Green Blue Violet White 
##### Blocks must be within 0.5” of one another and within 0.5” of a straight line
##### 15 points for each block in the correct order (5 if block is out of place)
##### Up to 30 points for the neatness of the line (judged by instructors)

**Event 4**: Stack ‘em high! (150 points)
##### Blocks will be placed in a specific configuration, some will be stacked (no more than 3 high)
##### Order The Blocks in a stack according to the resistor color code: 
##### Black Red Orange Yellow Green Blue Violet White 
##### 15 points for each block in the correct order (5 if block is out of space)
##### 5 points for each 20s less than 120s it takes to complete the stack (up to 30 points)

**Event 5**: Ancient Robotic Builder (200+ points)
##### Blocks will be placed randomly on the board
##### More blocks may be added at any time
##### The rexarm must build a 2D pyramid as wide and tall as it can, color does not matter
##### You have 300s (5m) to build the pyramid
##### Each complete level of the pyramid completed before the structure collapses, time runs out, or the bot can no longer place blocks will score 50 points. 


## Hardware Architecture

Hardware Architecture:
![alt text][image_hardware] 
Hardware Connection:
![alt text][image_hardware2] 

## Software Architecture
In this project, we use multi-process, multi-threaded system to handle different tasks simultaneously with much less time than normal single thread in main function. We create the software architecture to run and communicate the command more efficiently, and build the control decision making architecture to make the task running in a more logical and systematical way.

The software system could manipulate several robotics corporately in one or several remote computers during the same time and have the safe mechanism, both in communication channel check and subsystem condition check, to guarantee every task is executed successfully. Unless all the processes (on all the computers) or all the actuator (in this situation, the robotic arm) die, the task will be finished. This parallel computation framework is designed to handle much more heavy tasks in the same time, even we only use a small part of its potential in this project. 

* **Master process ** manages the jobs, distributes work and assigns them to worker process, and handles heartbeat check.

* **Worker process ** wait for commands from the master process, and then perform tasks based on given input arguments.

Software Architecture:
![alt text][image_multi_process_structure] 

The master process first creates a thread to initialize worker communication channel parameters and start a thread to build all the worker processes. Then the master will start job job listen thread to listen signals from worker process, and heartbeat listen thread to listen heartbeat from worker process. When master process creates new worker processes, these new processes start their own heartbeat thread, and keep listening on master TCP channel to receive command. Once worker process receives task command, they will execute these tasks and send task status feedback signal to master TCP listener. 

Functions logic chart:
![alt text][image_multi_function_chart] 

[Python main function](https://github.com/AaronWzh/vision_arm_robot/tree/master/rexarm_python). 

This system is able to run any job on our machine, using a actuator function (in this project, it is just motor control command) implementation.

A process is an executing program with a dedicated memory space. Many processes run at the same time on a computer. Processes have isolated memory spaces so one process cannot access the data of another process. Threads are similar to processes in that they allow for parallelization of work, but unlike processes, threads share the same memory space and can access each other’s data. Threads are owned and created by a single process, and are only alive as long as the parent process is alive. 

In this project, we create master process in `control_station.py` and start new processes in it. And we use TCP and UDP to communicate in different processes.

The system is designed to setup several processes in the same lab computer to do inverse kinematics calculation, and 7 motors (in arm) motion control separately, and continuously communicate with master server. The worker processes get the task, do the task, and give feedback to the master process, and the master process will just do the information arrangement and assign every work to the worker process. 

We were also considering all the image processing job in the worker process to reduce the workload of master process, but the GUI system in main process still need to handle the kinect image, so there would be a conflict if we try to call the kinect image obtaining function in different process at the same time. Since the image processing takes very short time (in our algorithm, less than 0.1s), we decided to keep the image processing part in main process. Since the speed bottleneck is the arm motion part, we put more energy in the dynamic motion planning.


##Communication
The communication in this system uses TCP (Transmission Control Protocol) and UDP (User Datagram Protocol). A socket creates and manages a TCP/UDP connection, and all sockets use a specific port. Sockets can be used to send data to a specific port, and to listen for data on a specific port. In this project, we use TCP for all communication on the main thread (like processes status listening, job assigning, and feedback sending), and UDP for all other communication (here, specifically heartbeat messages). 

`TCP` guarantees the recipient will receive the packets in order by numbering them. TCP is reliable. All packets sent with TCP are tracked so no data is lost or corrupted in transit. Packets using `UDP` are just sent to the recipient without error checking. The sender does not wait to make sure the recipient receive the packet. UDP is not very reliable when the communication noise is obvious, but it is faster than TCP. So in our project, we use TCP to mark the status of each process and do the job assignment and feedback return, and only use UDP to track heartbeat of each process.

For the communication data type, We use JSON as information package language for TCP and UDP. JSON (JavaScript Object Notation) is a lightweight data-interchange format. Since the JSON format is text only, it can easily be sent to and from a server, and used as a data format by any programming language.

All the massages in master and worker processes are packaged as a dictionary data structure we defined, and after jsonized, they will be sent and received in the TCP/UDP channel port.



## State Machine and Decision Making Architecture
State machine is implemented in master process, and it is for block picking and arranging. But the real execution step is in worker process level, with angle check and speed control. 

Decision making logic block:
![alt text][image_decision_making] 

The master process level (in red box) decide the structure of one complete step move:
##### 1) From initial location (zero point) go to the block location
##### 2) Grab target block
##### 3) Get to waypoint
##### 4) Get to transition waypoint
##### 5) Drop target block
##### 6) Modify gesture of gripper and orientation of block (this includes series actions)
##### 7) Grab block again
##### 8) Get to another waypoint
##### 9) Arrive final location (this includes series actions)
##### 10) Drop block
##### 11) Get to initial location

In step 6) and 9), we define several sub-steps to make sure the motion modification and block arrangement is smooth and accurate. For example, in the pyramid stacking task, when gripper puts the block in the final location, it first arrives above of the location, modifies its orientation, arrives the final location, drops block, moves above certain distance of the location again, and then goes to the next waypoint.

Each step above is defined in master process, and the command message is sent as a package through TCP to and executed by worker process. Once the action is finished, the worker process will return a certain message to notify the master process this step has been achieved and the next command can be send and executed.

For every worker process (in blue box), we define several basic commands:

* **arrive** : get to certain target point (x,y,z) by certain inverse kinematics method. Check if the action is achieved, and return a message to master.
* **grab** : grab target block. Check if the action is achieved by gripper motor servo torque and angle, and return a message to master.
* **drop** : drop target block. Check if the action is achieved by gripper motor servo angle, and return a message to master.
* **waypoint** : will get to waypoint. Similar to 'arrive', but we don't need target point coordinate. The waypoint is automatically calculated by current end effector location and angle. Check if the action is achieved, and return a message to master.
* **cmd** : will give direct commend to motor. Check if the action is achieved, and return a message to master. 

In this three level decision making architecture, we package basic command and state machine combo in different level, and for the final task running level, we can easily just call the action in package to execute the strategy level move. 

## Gripper design
Gripper is a very important part of design to do tasks efficiently. We designed two kinds of grippers, a four bar gripper and a direct motor drive gripper. 

Gripper No.1:
![alt text][image_gripper]

The four bar one can always be parallel to the blocks easily for us to calibrate the grasp and drop angle. And the claw can move forward when gripping, which can compensate the tolerance of localization. But during the experimentation, we find out that XL-320 motor has very limited torque and OLLO rivet and hole mate are generating relatively large friction force. So when trying to grasp the block, it may overload sometime. Also when using the swap mode to grasp the block, the four bar gripper will take up extra space, which may collide with other blocks. So we designed a second gripper.  

Gripper No.2:
![alt text][image_gripper]
The second gripper is driven directly by motor without any middle mechanism part except for gear mate. In the test, this new design fits our grasp mode very well. It can save space up to 40mm along the z axis of the frame fixed on the gripper. And this compact gripper can make the most of the XL-320 torque to grasp more tightly than its predecessor. Additionally, having 2 extra DOF provides more freedom and is perfectly suitable with three different gripping mode (these will be introduced later in inverse kinematic section) which we implement in our target acquisition strategy.

## Forward Kinematics
Forward kinematics refers to the use of the kinematic equations of a robot to determine the position and orientation of the end effector given the values for the joint angle variables of the arm robot. We establish the relationship between the base fixed frame and the end effector fixed frame. A commonly used convention for selecting frames of reference for robots is the Denavit-Hartenberg (DH) convention, a convention for the definition of the joint matrices and link matrices to standardize the coordinate frame for spatial linkages. When applying this convention to the arm robot, we can easily get the homogeneous transformation matrix connecting two adjacent links, which consist of four basic transformation matrices. 
DH table:
![alt text][image_DH_table]

## Model for Inverse Kinematics
In most cases, we want the arm robot to move to the given position to pick up blocks. Then we need inverse kinematics to determine the values of the joint angle variables given the end effector’s position and orientation. In this case, we can use geometric decoupling to consider the position and orientation problems independently, and we can get closed form solution. Due to our gripper’s flexibility with two additional wrist DOF, we design three inverse kinematics modes to make the most of the gripper in order to augment the robot’s workspace to meet different task requirements.

3 modes: Left:sweep;Middle:perpendicular;Right:free pick mode:
![alt text][image_3modes]

## Computer Vision

We use opencv to detect the blocks and get corresponding information of a specific block. First, image captured from video is converted from RGB to HSV. HSV (Hue, Saturation, Value) is a way of encoding color images. It is sensitive for computer to segment image based on the color of the objects. Second, we use `cv2.inRange()` function in opencv to produces a binary image setting pixels in the range specified to white, and pixels outside to black. Next, morphological operations are performed on binary images to efficiently remove noise in binary images. We use `cv2.morphologyEx()` to remove noise and join broken parts of an object, which is same as erosion followed by dilation. In this way, we are able to detect different colors in high accuracy. Then we use `cv2.findcontours()` to get all the contour information of the specific block as well as the pixel coordinate of the centroid of the block. To increase the accuracy of detection and enhance the robustness of the detector, we use area, perimeter and corner number to check whether it’s the block we want to detect. What's more, we use `cv2.bitwise_and()` to filter the image information outside the greyboard by setting pixels outside to black.

## Path Smoothing

If a command is given to make the arm move to a specific point, every joint will move directly to the desired angle. But sometimes we need to find a trajectory connect initial and final configuration to satisfy the velocity and acceleration constraints, and make the move smooth. In the teach and repeat task we use cubic spline method. There are basically two method to generate cubic spline: joint angle and end point of arm. We adopt method of generating cubic spline of joint angles, and use forward kinematics to get the end point coordinates. This can sample enough waypoints to make the motor move in a smooth trajectory and avoid angle go-stop spike.

## Task Completion Strategy
To explore fast but effective way to complete transitions from picking and placing any block, about 20 steps are recognized and manually assigned for tuned torque and speed. However, strategy used is specific to the tasks at hand. A total of five different tasks are performed:

* **Task 1 (Pick and Place)**: This task requires us to move 3 different blocks from their original positions with some (x,y) coordinates, which are generated through blob detection to the target position having (x,-y) coordinates with X-axis acting as the mirror. Since the generalized block dimensions are known and the blocks are oriented towards center (center of the bottom of arm), we aligned the arm in direction of block orientation with rotating base motor while maintaining initial position for remaining joints. Using inverse kinematics algorithm, gripper picks up the block and returns to initial position where it aligns itself to the target position using base motor. Basic maths operation flips the Y - Axis and save that as target position. Using Inverse kinematics, gripper drops the block at target position.

* **Task 2 (Pick and Stack)**: This task requires 3 blocks to be stacked in a particular order. Same strategy is employed for picking up the blocks as task 1. After picking up the blocks, they are placed at a specific location with same (x,y) coordinates but with a different Z - coordinate (also taking into account the offset generated because of the mechanical parts via observations), which is 38.5 mm (distance between 2 block centers) greater than previous one.

* **Task 3 (Line them up)**: This task requires blocks to be placed in a line with order according to required color code: Black, Red, Orange, Yellow, Green, Blue, Violet, White. They are initially placed in a specific configuration, some stacked (no more than 3 high). Using the RGBD output from Kinect camera, we calculate the coordinates and depth of each block. If the blocks appear to be stacked, we unstack each block and place them at specific position in safe zone (knowing that there won't be any block in safe zone). Then using computer vision blob detection we identify the color as well as coordinates of each block. Using Inverse Kinematics we pick up first block (Black) and placed it at a specific predefined (x,y,z) position. For next block placing, the Y and Z Coordinates remain constant while X coordinate is increased with a factor (Delta + width of the cubic block), where delta is the distance we assign between 2 blocks placed in line. However, there is an intermediate transition stage just after picking up block, where the block is placed (at predefined position in safe zone) for re-orientation by 90°(counter clockwise along Z axis), to minimize the error in block orientation generated while picking up.

* **Task 4 (Stack them High)**: Same initial configuration as task 3 is given with some blocks being stacked (no more than 3 high). Blocks need to be stacked on top of each other in a certain order, as the required color code: Black, Red, Orange, Yellow, Green, Blue, Violet, White. Same blob detection and depth calculating strategy is used to unstack and identify the color of the blocks. After all the blocks are finally placed in fixed predefined (X,Y) coordinates, we start to stack. The Z-coordinates for every next block is increased by a factor equal to the width of the cubic block (38mm) with certain compensation error measured in experiment. Intermediate stage (as explained in task 3) is used for correcting orientation of gripper and block and minimizing errors from initial block orientation while picking.

* **Task 5 (Pyramid Builder)**: In this task, blocks are placed randomly on the board. More blocks are added at any time if needed. Final position should resemble a 2D pyramid as wide and tall as it can, regardless to color specification. To make this task fast, we eliminate the color recognition task and only utilize blob detection to calculate the position of block. During training, the coordinates for every consecutive block placing is recorded with respect to the center of the arm. This task also use the intermediate transition stage (mentioned in task 3 and 4) for greater accuracy. 

## Performance
Performance and Scores:
![alt text][image_performance]

## Acknowledgment
This project is based on ROB550 project in University of Michigan.
Code contributors: Zihang Wei (wzih@umich.edu), Cong Fu (congfu@umich.edu)
We would like to thank Prof. Ella Atkins, course instructor, for scientific guidance, Dr. Peter gaskell, lab instructor, for providing us with the proper equipment set and guidance throughout the experimentation process as well as lab sessions.