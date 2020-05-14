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
Publishers:
- "path_segments": mav_planning_msgs::PolynomialTrajectory <sup>1</sup>
- "path_segments_4D": mav_planning_msgs::PolynomialTrajectory4D <sup>1</sup>

<sup>1</sup> Either will be remapped to "trajecotry" depending on if yaw is included in key waypoints.

Subscribers:
- "/mavros/local_position/pose": geometry_msgs::PoseStamped
- "waypoints": uav_motion::waypointsGoal <sup>2</sup>

<sup>2</sup> It is a ROS action server.

Parameters:
- "mav_v": maximum velocity
- "mav_a": maximum acceleration
- "mav_ang_v": maximum angular velocity
- "mav_ang_a": maximum angular acceleration
- "current_pose_as_start": the current pose will be included as the start point of trajectory if true

### 2. trajectory_sampler
Publishers:
- "reference/flatsetpoint": controller_msgs::FlatTarget
- "reference/yaw": std_msgs::Float32

Subscribers:
- "path_segments": mav_planning_msgs::PolynomialTrajectory <sup>1</sup>
- "path_segments_4D": mav_planning_msgs::PolynomialTrajectory4D <sup>1</sup>

Parameters:
- "dt": trajectory sampling rate

### 3. geometric_controller
Publishers:
- "/command/bodyrate_command" -> "/mavros/setpoint_raw/attitude": mavros_msgs::AttitudeTarget

Subscribers:
- "reference/flatsetpoint": controller_msgs::FlatTarget
- "reference/yaw": std_msgs::Float32

More information about this node can be found on [Jaeyoung-Lim
/mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers).

### 4. waypoint_generator.py
Publisher:
- "waypoints": uav_motion.msg.waypointsAction <sup>3</sup>

<sup>3</sup> It is a ROS action client.
