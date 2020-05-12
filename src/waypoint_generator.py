#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import uav_motion.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import numpy as np

def waypoints_client(positions, yaws):
    client = actionlib.SimpleActionClient('waypoints', uav_motion.msg.waypointsAction)
    client.wait_for_server()
    goal = uav_motion.msg.waypointsGoal()
    assert positions.shape[0] == len(yaws)
    for i in range(len(yaws)):
        p = positions[i]
        yaw = yaws[i]
        q = quaternion_from_euler(0, 0, yaw)
        pose = geometry_msgs.msg.Pose()
        pose.position.x = float(p[0])
        pose.position.y = float(p[1])
        pose.position.z = float(p[2])
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        goal.poses.append(pose)
    
    client.send_goal(goal)
    print("goal sent")
    client.wait_for_result()
    return client.get_result()
    
    
    
if __name__ == '__main__':
    rospy.init_node('waypoints_client', anonymous=False)
    positions = np.asarray(((0, 0, 5), (4, 3, 5), (-2, 6, 5), (-4, -2, 5)))
    yaws = [0, 0.5, -0.5, 0]
    result = waypoints_client(positions, yaws)
    print(result)