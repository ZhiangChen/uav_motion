/*
 * Copyright (c) 2017, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Helen Oleynikova, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Rik Bähnemann, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Marija Popovic, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Modified by Zhiang Chen, May 2020
 * Apache License Version 2.0
 * */
#include <uav_motion/trajectory_sampler.h>

TrajectorySamplerNode::TrajectorySamplerNode(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private):
nh_(nh),
nh_private_(nh_private),
dt_(0.01),
current_sample_time_(0.0)
{
	ros::Duration(0.5).sleep();
	nh_private_.param("/trajectory_sampler/dt", dt_, dt_);

	command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
	trajectory_sub_ = nh_.subscribe("path_segments", 10, &TrajectorySamplerNode::pathSegmentsCallback, this);
	trajectory4D_sub_ = nh_.subscribe("path_segments_4D", 10, &TrajectorySamplerNode::pathSegments4DCallback, this);
	stop_srv_ = nh_.advertiseService("stop_sampling", &TrajectorySamplerNode::stopSamplingCallback, this);
	position_hold_client_ = nh_.serviceClient<std_srvs::Empty>("back_to_position_hold");

	flatreferencePub_ = nh_.advertise<controller_msgs::FlatTarget>("reference/flatsetpoint", 1);
	yawreferencePub_ = nh_.advertise<std_msgs::Float32>("reference/yaw", 1);

	const bool oneshot = false;
	const bool autostart = false;
	publish_timer_ = nh_.createTimer(ros::Duration(dt_), &TrajectorySamplerNode::commandTimerCallback, this, oneshot, autostart);
}

TrajectorySamplerNode::~TrajectorySamplerNode()
{
	publish_timer_.stop();
}

void TrajectorySamplerNode::pathSegmentsCallback(const mav_planning_msgs::PolynomialTrajectory&
		segments_message)
{
	if (segments_message.segments.empty())
	{
		ROS_WARN("Trajectory sampler: received empty waypoint message");
		return;
	}
	else
	{
		ROS_INFO("Trajectory sampler: received %lu new waypoints", segments_message.segments.size());
	}

	bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(segments_message,
			&trajectory_);
	if (!success)
	{
		return;
	}
	processTrajectory();
}

void TrajectorySamplerNode::pathSegments4DCallback(const mav_planning_msgs::PolynomialTrajectory4D&
		segments_message)
{
	if (segments_message.segments.empty())
	{
		ROS_WARN("Trajectory sampler: received empty waypoint message");
		return;
	}
	else
	{
		ROS_INFO("Trajectory sampler: received %lu waypoints", segments_message.segments.size());
	}

	bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(segments_message,
			&trajectory_);
	if (!success)
	{
		return;
	}
	processTrajectory();
}

void TrajectorySamplerNode::processTrajectory()
{
	// Call the service call to takeover publishing commands.
	if (position_hold_client_.exists())
	{
		std_srvs::Empty empty_call;
		position_hold_client_.call(empty_call);
	}

	if (publish_whole_trajectory_)
	{
		// Publish the entire trajectory at once.
		mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
		mav_trajectory_generation::sampleWholeTrajectory(trajectory_, dt_,
														 &trajectory_points);
		trajectory_msgs::MultiDOFJointTrajectory msg_pub;
		msgMultiDofJointTrajectoryFromEigen(trajectory_points, &msg_pub);
		command_pub_.publish(msg_pub);
	}
	else
	{
		publish_timer_.start();
		current_sample_time_ = 0.0;
		start_time_ = ros::Time::now();
	}
}

bool TrajectorySamplerNode::stopSamplingCallback(std_srvs::EmptyRequest& request,
		std_srvs::EmptyResponse& response)
{
	publish_timer_.stop();
	return true;
}

void TrajectorySamplerNode::commandTimerCallback(const ros::TimerEvent&)
{
	if (current_sample_time_ <= trajectory_.getMaxTime())
	{
		trajectory_msgs::MultiDOFJointTrajectory msg;
		mav_msgs::EigenTrajectoryPoint trajectory_point;
		bool success = mav_trajectory_generation::sampleTrajectoryAtTime(trajectory_, current_sample_time_,
				&trajectory_point);
		if (!success)
		{
			  publish_timer_.stop();
		}
		mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
		msg.points[0].time_from_start = ros::Duration(current_sample_time_);
		msg.header.stamp = ros::Time::now();
		//command_pub_.publish(msg);
		current_sample_time_ += dt_;


		controller_msgs::FlatTarget traj_msg;
		traj_msg.type_mask = 2;
		traj_msg.header = msg.header;
		traj_msg.position = msg.points[0].transforms[0].translation;
		traj_msg.velocity = msg.points[0].velocities[0].linear;
		traj_msg.acceleration = msg.points[0].accelerations[0].linear;
		geometry_msgs::Quaternion quat = msg.points[0].transforms[0].rotation;
		double yaw = tf::getYaw(quat);
		std_msgs::Float32 yaw_msg;
		yaw_msg.data = static_cast<float>(yaw);

		flatreferencePub_.publish(traj_msg);
		yawreferencePub_.publish(yaw_msg);


	}
	else
	{
		publish_timer_.stop();
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "trajectory_sampler");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	TrajectorySamplerNode trajectory_sampler_node(nh, nh_private);
	ROS_INFO("trajectory_sampler has been initialized!");
	ros::spin();
}
