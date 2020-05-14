# uav_motion

This repository generates a minimum-snap polynomial trajectory and controls a quadrotor with PX4 framework. It depends on [ethz-asl/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation) and [Jaeyoung-Lim
/mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers).

# Installation
1. ethz-asl/mav_trajectory_generation
```
cd ~/catkin_ws/src
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/eigen_catkin.git
git clone https://github.com/ethz-asl/eigen_checks.git
git clone https://github.com/ethz-asl/nlopt.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/ethz-asl/mav_comm.git
git clone https://github.com/ethz-asl/yaml_cpp_catkin.git
git clone https://github.com/ethz-asl/mav_trajectory_generation.git
cd ~/catkin_ws/
catkin build
```

2. Jaeyoung-Lim/mavros_controllers
```
cd ~/catkin_ws/src
https://github.com/Jaeyoung-Lim/mavros_controllers.git
cd ~/catkin_ws/
catkin build
```

# Getting Started
This gives you an example of using this package in gazebo. 

1. Launch a quadrotor with px4 and mavros in gazebo 
```
roslaunch px4 posix_sitl.launch
```
2. Define your keyframe waypoints in waypoint_generator.py and run the following commands
```
roslaunch uav_motion uav_motion.launch
rosrun uav_motion waypoint_generator.py
```

# ROS Nodes
### 1. trajectory_generator

### 2. trajectory_sampler


### 3. geometric_controller

