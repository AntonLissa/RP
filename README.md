# Arm Simulation ROS2

A simple ROS2 C++ simulation of a robotic arm with forward and inverse kinematics, publishing joint states and trajectory markers to visualize motion in RViz.
![3R simulation in RViz](screen1.png)
![2R simulation in RViz](screen2.png)
## Features
- Forward and inverse kinematics with Jacobian-based IK
- Trajectory generation: circle, ellipse, or custom points
- RViz visualization of arm motion and trajectory
- Configurable DH file and update rate


## Run the code
```ros2 launch arm_sim arm_full_launch.py```
