# hand-eye-calibration
This eye-in-hand calibration project aims at making a vision module for pose estimation using the cameras added into a UR3 robot gripper grippers. 

The system as well as the calibration calculations are described by the following figure. 

<img src=data/system-calc.png>

## The calibration reference pose
This figure shows the reference pose used for the calibration. To calculate one arm's camera-to-end effector transformation, an AR marker is **put at the exact pose** of the other arm's end effector.

<img src=data/calib-pose.jpg>

## Result
Some of the results of 3D pose estimation of an electric saw, as well as an electric screw driver. The real task start and goal poses and their corresponding estimated poses in the simulation environment are shown below.

<img src=data/saw-start-goal-real-sim.png>

<img src=data/elec-drive-real-sim.png>

## Precision screwing task
In this screwing task, the robot learned the start and goal poses using the result of the eye-in-hand calibration process.

<img src=data/screwingx2.gif>


The details of this vision module are described in [this paper](https://ieeexplore.ieee.org/document/8674857). This module was integrated in my lab's robotics development environment as a method for 3D pose estimation. 

## Contact
If you are interested in the presented work/ideas, or if you have any questions, feel free to connect with me on [LinkedIn](https://www.linkedin.com/in/mohraess). We can disuss about this project and other interesting projects.
