#!/usr/bin/env python3
"""
Zhiang Chen
"""

import rospy

import dynamic_reconfigure.client


def update_Kp_z(Kp_z):
	params = {'max_acc': 8.0, 'Kp_x': 8.0, 'Kp_y': 8.0, 'Kp_z': 10., 'Kv_x': 1.5, 'Kv_y': 1.5, 'Kv_z': 3.3, 'groups': {'id': 0, 'parent': 0, 'name': 'Default', 'type': '', 'state': True, 'groups': {}, 'parameters': {}, 'max_acc': 8.0, 'Kp_x': 8.0, 'Kp_y': 8.0, 'Kp_z': 4.975, 'Kv_x': 1.5, 'Kv_y': 1.5, 'Kv_z': 3.3}}
	client = dynamic_reconfigure.client.Client("geometric_controller")
	params['Kp_z'] = Kp_z	
	client.update_configuration(params)

if __name__ == "__main__":
	rospy.init_node("dynamic_client")
	update_Kp_z(6)
	
