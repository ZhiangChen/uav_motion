#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import uav_motion.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import numpy as np
from std_srvs.srv import Empty

def waypoints_client(positions, yaws):
    client = actionlib.SimpleActionClient('waypoints', uav_motion.msg.waypointsAction)
    client.wait_for_server()
    goal = uav_motion.msg.waypointsGoal()
    assert positions.shape[0] == len(yaws)
    for i in range(len(yaws)):
        p = positions[i]
        yaw = yaws[i]
        position = geometry_msgs.msg.Point(p[0], p[1], p[2])
        
        goal.positions.append(position)
        goal.yaws.append(yaw)
    
    client.send_goal(goal)
    print("goal sent")
    client.wait_for_result()
    return client.get_result()
    
    
if __name__ == '__main__':
    rospy.init_node('waypoints_client', anonymous=False)
    rospy.wait_for_service('stop_sampling')
    stop_srv_client_ = rospy.ServiceProxy('stop_sampling', Empty)
    positions = np.asarray(((0, 0, 10), (4, 4, 10), (0, 8, 10), (-4, 4, 10), (0, 0, 10), (4, -4, 5), (0, -8, 5), (-4, -4, 5), (0, 0, 5)))
    yaws = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    #positions = np.asarray(((0, 0, 1.5), (-10, -10, 2), (0, 0, 1)))
    #yaws = [-0.5, -0., -0.5]

    #positions = np.asarray(( (-13.5, 0, 10), (-15, 0, 10),))
    #yaws = [-3, -3]
    #positions = np.asarray(((-18, 0, 11), (-18, 1, 11), (-10, 1, 11)))
    #yaws = [-3, 0, 0]
    result = waypoints_client(positions, yaws)
    #rospy.sleep(5.)
    #positions = np.asarray(((-4, 4, 10), (0, 0, 10), (4, -4, 5), (0, -8, 5), (-4, -4, 5), (0, 0, 5)))
    #yaws = [3, 4, 5, 6, 7, 8]
    #result = waypoints_client(positions, yaws)
    #rospy.sleep(5.)
    #positions = np.asarray(((0, -8, 5), (-4, -4, 5), (0, 0, 5)))
    #yaws = [6, 7, 8]
    #result = waypoints_client(positions, yaws)
