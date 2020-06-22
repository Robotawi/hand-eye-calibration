# hand-eye-calibration
This Eye-in-Hand calibration code aims at making a vision module for pose estimation using the cameras integrated into a UR3 robot gripper. 

The system as well as the calibration calculations are described by the following figure. 

<img src=data/system-calc.png>

## The calibration reference pose
This figure shows the reference pose used for the calibration. To calculate one arm's camera-to-end effector transformation, an AR marker is put at the exact pose of the other arm.

<img src=data/calib-pose.jpg>

## Result
Some of the results of 3D pose stimation of an electric saw, and an electric screw driver. The real start and goal poses and their corresponding estimated poses in the simulation are shown.

<img src=data/saw-start-goal-real-sim.png>

<img src=data/elec-drive-real-sim.png>

## Precision screwing task
Screwing task in which the robot learned the start and goal poses using the result of the calibration process.

<img src=data/screwingx2.gif>


The details of the hand-eye-calibration module are described in [this paper](https://ieeexplore.ieee.org/document/8674857). This module was integrated in my lab's robotics development environemt as a method for 3D pose estimation. 

## Contact
If you are interested in the presented work/ideas, or if you have any questions, feel free to connect with me on [LinkedIn](https://www.linkedin.com/in/mohraess). We can disuss about this project and other interesting projects.
