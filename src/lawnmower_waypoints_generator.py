#!/usr/bin/env python3
"""
Zhiang Chen
"""
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
import uav_motion.msg
import actionlib
import numpy as np
from tf.transformations import euler_from_quaternion
from param_update import update_Kp_z



class PathPlanner(object):
    def __init__(self):
        self.current_pose_ = PoseStamped()
        self.current_pose_.pose.orientation.w = 1
        self.saved_pose_ = PoseStamped()
        self.goal_position_ = Point()

        self.poses = []
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.local_path_pub = rospy.Publisher("/local_path", Path, queue_size=1)

        self.client_ = actionlib.SimpleActionClient('waypoints', uav_motion.msg.waypointsAction)
        self.client_.wait_for_server()

        current_pose_sub_ = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback,
                                             queue_size=1)

        rospy.loginfo("Path planner has been initialized!")


    def startSearch(self):
        positions = np.asarray(((0, 0, 3), (-2, 0, 3), (-2, -12, 3), (3.5, -12, 3), (3.5, -10, 3), (-2, -10, 3), (-2, -8, 3), (3.5, -8, 3), (3.5, -6, 3), (-2, -6, 3),  (0,0,3)))
        yaws = self.getHeads(positions)

        assert positions.shape[0] == len(yaws)

        for i in range(len(yaws)):
            goal = uav_motion.msg.waypointsGoal()
            goal_p = positions[i]

            self.goal_position_.x = float(goal_p[0])
            self.goal_position_.y = float(goal_p[1])
            self.goal_position_.z = float(goal_p[2])
            q = self.current_pose_.pose.orientation
            yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
            self.goal_yaw_ = yaw

            goal.positions.append(self.goal_position_)
            goal.yaws.append(yaw)
            self.client_.send_goal(goal)
            while True & (not rospy.is_shutdown()):
                rospy.sleep(1.)
                current_p = np.asarray((self.current_pose_.pose.position.x,
                                        self.current_pose_.pose.position.y,
                                        self.current_pose_.pose.position.z))
                dist = np.linalg.norm(goal_p - current_p)

                if dist < 0.25:
                    break

            rospy.sleep(1.)
            goal = uav_motion.msg.waypointsGoal()
            goal.positions.append(self.goal_position_)
            goal.yaws.append(yaws[i])
            self.client_.send_goal(goal)
            rospy.sleep(5.)

    def getHeads(self, waypoints):
        yaws = []
        nm = waypoints.shape[0]
        for i in range(nm-1):
            currnt_p = waypoints[i][:2]
            nxt_p = waypoints[i+1][:2]
            dir = nxt_p - currnt_p
            yaws.append(np.arctan2(dir[1], dir[0]))

        yaws.append(0)
        return yaws

    def poseCallback(self, pose):
        self.current_pose_ = pose
        self.poses.append(pose)
        self.path.poses = self.poses
        self.local_path_pub.publish(self.path)



if __name__ == '__main__':
    rospy.init_node('waypoints_client', anonymous=False)
    update_Kp_z(8)
    path_planner = PathPlanner()
    path_planner.startSearch()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node killed!")

