#include "uav_motion/trajectory_generator.h"

template <int _N>
TrajectoryGenerator<_N>::TrajectoryGenerator(ros::NodeHandle& nh, int dimension):
nh_(nh),
as_(nh_, "waypoints", boost::bind(&TrajectoryGenerator::waypointsCallback, this, _1), false),
current_velocity_(Eigen::Vector3d::Zero()),
current_angular_velocity_(Eigen::Vector3d::Zero()),
current_pose_(Eigen::Affine3d::Identity()),
dimension_(dimension),
opt_ptr_(new mav_trajectory_generation::PolynomialOptimization<_N>(dimension)),
max_v_(2.0),
max_a_(2.0),
max_ang_v_(1.0),
max_ang_a_(1.0)
{
	nh_.param<double>("max_v", max_v_, max_v_);
	nh_.param<double>("max_a", max_a_, max_a_);
	nh_.param<double>("max_ang_v", max_ang_v_, max_ang_v_);
	nh_.param<double>("max_ang_a", max_ang_a_, max_ang_a_);

	pub_trajectory_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory>("trajectory", 0);
	sub_local_pose_ = nh_.subscribe("/mavros/local_position/pose", 1,
			&TrajectoryGenerator::uavLocalPoseCallback, this);
	sub_local_vel_ = nh_.subscribe("/mavros/local_position/velocity", 1,
				&TrajectoryGenerator::uavVelocityCallback, this);
	as_.start();

	ROS_INFO("trajectory_generator has been initialized!");
}

template <int _N>
TrajectoryGenerator<_N>::~TrajectoryGenerator(void){}

template <int _N>
void TrajectoryGenerator<_N>::uavLocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	tf::poseMsgToEigen(pose->pose, current_pose_);
}

template <int _N>
void TrajectoryGenerator<_N>::uavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& vel)
{
	tf::vectorMsgToEigen(vel->twist.linear, current_velocity_);
	tf::vectorMsgToEigen(vel->twist.angular, current_angular_velocity_);
}

template <int _N>
void TrajectoryGenerator<_N>::waypointsCallback(const uav_motion::waypointsGoalConstPtr& goal)
{
	waypoints_ = goal->poses;
	mav_trajectory_generation::Vertex::Vector vertices;

	for(int i=0; i<waypoints_.size(); i++)
	{
		mav_trajectory_generation::Vertex point(dimension_);
		if (i==0 || i==(waypoints_.size()-1))  // start point and end point
		{
			if(dimension_ == 3)
			{
				geometry_msgs::Point position = waypoints_[i].position;
				double x = position.x;
				double y = position.y;
				double z = position.z;
				point.makeStartOrEnd(Eigen::Vector3d(x, y, z), derivative_to_optimize);
			}
			else if(dimension_ == 4)
			{
				geometry_msgs::Point position = waypoints_[i].position;
				double x = position.x;
				double y = position.y;
				double z = position.z;
				geometry_msgs::Quaternion quat = waypoints_[i].orientation;
				double yaw = tf::getYaw(quat);
				point.makeStartOrEnd(Eigen::Vector4d(x, y, z, yaw), derivative_to_optimize);
			}
			else
			{
				result_.success = false;
				as_.setSucceeded(result_);
				return;
			}


		}
		else  // middle points
		{
			if(dimension_ == 3)
			{
				geometry_msgs::Point position = waypoints_[i].position;
				double x = position.x;
				double y = position.y;
				double z = position.z;
				point.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
						Eigen::Vector3d(x, y, z));
			}
			else if(dimension_ == 4)
			{
				geometry_msgs::Point position = waypoints_[i].position;
				double x = position.x;
				double y = position.y;
				double z = position.z;
				geometry_msgs::Quaternion quat = waypoints_[i].orientation;
				double yaw = tf::getYaw(quat);
				point.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
						Eigen::Vector4d(x, y, z, yaw));
			}
			else
			{
				result_.success = false;
				as_.setSucceeded(result_);
				return;
			}
		}
		vertices.push_back(point);
	}
	std::vector<double> segment_times;
12
	segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

	opt_ptr_->setupFromVertices(vertices, segment_times, derivative_to_optimize);
	opt_ptr_->solveLinear();

	mav_trajectory_generation::Segment::Vector segments;
	opt_ptr_->getSegments(&segments);

	mav_trajectory_generation::Trajectory trajectory;
	opt_ptr_->getTrajectory(&trajectory);

	mav_planning_msgs::PolynomialTrajectory msg;
	mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
																	 &msg);
	msg.header.frame_id = "world";

	pub_trajectory_.publish(msg);

	result_.success = true;
	as_.setSucceeded(result_);
}



int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "trajectory_generator");
	ros::NodeHandle nh;

	const int N = 10;
	int dimension = 4;
	TrajectoryGenerator<N> traj_gen(nh, dimension);

	ros::spin();
	 	
 	std::cout<<"Done!"<<std::endl;
	return 0;
}

