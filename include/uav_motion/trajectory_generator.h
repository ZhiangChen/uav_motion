#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <ros/ros.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/trajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>

#include <actionlib/server/simple_action_server.h>
#include <uav_motion/waypointsAction.h>

template <int _N = 10>
class TrajectoryGenerator
{
public:
	TrajectoryGenerator(ros::NodeHandle& nh, int dimension);
	~TrajectoryGenerator(void);

	void uavLocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);
	void uavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& vel);
	void waypointsCallback(const uav_motion::waypointsGoalConstPtr& goal);

protected:
	ros::NodeHandle& nh_;
	ros::Publisher pub_trajectory_;
	ros::Publisher pub_trajectory4d_;
	ros::Subscriber sub_local_pose_;
	ros::Subscriber sub_local_vel_;
	actionlib::SimpleActionServer<uav_motion::waypointsAction> as_;
	uav_motion::waypointsFeedback feedback_;
	uav_motion::waypointsResult result_;
	std::vector<geometry_msgs::Pose> waypoints_;

	Eigen::Affine3d current_pose_se3_;
	Eigen::Vector3d current_velocity_;
	Eigen::Vector3d current_angular_velocity_;
	double max_v_; // m/s
	double max_a_; // m/s^2
	double max_ang_v_;
	double max_ang_a_;

	const int dimension_;
	mav_trajectory_generation::PolynomialOptimization<_N>* opt_ptr_;
	const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::SNAP;
	bool current_pose_as_start_;

	bool addStartOrEnd_(geometry_msgs::Pose pose, mav_trajectory_generation::Vertex::Vector& vertices);
	bool addMiddle_(geometry_msgs::Pose pose, mav_trajectory_generation::Vertex::Vector& vertices);

};


#endif
