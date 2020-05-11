#include "uav_motion/trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle& nh):
nh_(nh),
as_(nh_, "waypoints", boost::bind(&TrajectoryGenerator::waypointsCallback, this, _1), false),
current_velocity_(Eigen::Vector3d::Zero()),
current_angular_velocity_(Eigen::Vector3d::Zero()),
current_pose_(Eigen::Affine3d::Identity())
{
	pub_trajectory_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory>("trajectory", 0);
	sub_local_pose_ = nh_.subscribe("/mavros/local_position/pose", 1,
			&TrajectoryGenerator::uavLocalPoseCallback, this);
	sub_local_vel_ = nh_.subscribe("/mavros/local_position/velocity", 1,
				&TrajectoryGenerator::uavVelocityCallback, this);
}

TrajectoryGenerator::~TrajectoryGenerator(void){}

void TrajectoryGenerator::uavLocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	tf::poseMsgToEigen(pose->pose, current_pose_);
}

void TrajectoryGenerator::uavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& vel)
{
	tf::vectorMsgToEigen(vel->twist.linear, current_velocity_);
	tf::vectorMsgToEigen(vel->twist.angular, current_angular_velocity_);
}

void TrajectoryGenerator::waypointsCallback(const uav_motion::waypointsGoalConstPtr& goal)
{

}



int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "trajectory_generator");
	ros::NodeHandle nh;

	TrajectoryGenerator traj_gen(nh);

	ros::Publisher pub_trajectory = nh.advertise<mav_planning_msgs::PolynomialTrajectory>("trajectory",
                                                              0);
	
	mav_trajectory_generation::Vertex::Vector vertices;
	const int dimension = 3;
	const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
	mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

	start.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);
	vertices.push_back(start);
	
	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,2,3));
	vertices.push_back(middle);
	
	end.makeStartOrEnd(Eigen::Vector3d(2,1,5), derivative_to_optimize);
	vertices.push_back(end);
	
	std::vector<double> segment_times;
	const double v_max = 2.0;
	const double a_max = 2.0;
	segment_times = estimateSegmentTimes(vertices, v_max, a_max);
 	
 	const int N = 10;
	mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
	opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
	opt.solveLinear();
	
	mav_trajectory_generation::Segment::Vector segments;
	opt.getSegments(&segments);
	
	mav_trajectory_generation::Trajectory trajectory;
	opt.getTrajectory(&trajectory);
	
	mav_planning_msgs::PolynomialTrajectory msg;
	mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
	                                                                 &msg);
	msg.header.frame_id = "world";
	
	ros::Rate loop_rate(10);
	
	for (int i=0; i<50; i++)
	{
		loop_rate.sleep();
	}
	pub_trajectory.publish(msg);
	/*
	while(ros::ok())
	{
		//ROS_INFO_STREAM(msg);
		pub_trajectory.publish(msg);
		loop_rate.sleep();
	}
	*/
	
	
	 	
 	std::cout<<"Done!"<<std::endl;
	return 0;
}

