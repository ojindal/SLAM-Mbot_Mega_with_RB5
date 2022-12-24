# SLAM implemented on Mbot Mega Robot on Qualcomm RB5 Platform

<p align="justify">
This project focuses on implementation of Kalman Filter, a version of the Simultaneous Localization and Mapping (SLAM) technique and evaluation of its performance over variations in robot trajectory.
</p>

## Project Report
[Orish Jindal, Sanchit Gupta, 'Implementation of Kalman Filter and evaluation of its performance over variations in robot trajectory.
', CSE 276A, Course Project, UCSD](https://github.com/ojindal/SLAM-Mbot_Mega_with_RB5/blob/main/Orish%20-%20HW3.pdf)


## The actual environment containg AprilTags as landmarks.
<p align="center">
  <img width="500" alt="image" src="https://user-images.githubusercontent.com/89351094/209446533-09e7b24f-2364-4238-b852-8e6ccfaf9075.png">
 </p>
 
 ## Map of the environment created when the robot is run in an octagonal path multiple times.
<p align="center">
  <img src = "https://user-images.githubusercontent.com/89351094/209446604-b6ee7b2f-303f-4f5c-8ce1-b9a48fd4ffdc.png"/>
</p>

## Details to run the code

The architecture of the code includes the following four nodes:

* <b> Node 1: </b> Camera node initialized using rb5_camera_main_ocv.launch executable
* <b> Node 2: </b> April tag detection node initialized using apriltag_detection_array.py executable
* <b> Node 3: </b> MPI control node initialized using hw2_mpi_control_node.py executable
* <b> Node 4: </b> SLAM node initialized rosrun rb5_control slam.py
* <b> Node 5: </b> Path planning node initialized using hw4_sol.py executable
