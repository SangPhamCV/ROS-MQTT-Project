#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.client import Client

def set_local_planner_params():
    rospy.init_node('set_local_planner_params')

    # Specify the name of the dynamic reconfigure server for the local planner
    client = Client("/move_base/DWAPlannerROS")

    # Example parameters to set, replace with actual parameter names and values
    params = {
        'max_vel_x': 0.2,
        'min_vel_x': 0.2,
        'max_vel_theta': 1.82,
        'min_vel_theta': 0.9,
        # Add more parameters as needed
    }

    # Update the configuration
    client.update_configuration(params)

    rospy.loginfo("Local planner parameters set successfully")

if __name__ == '__main__':
    try:
        set_local_planner_params()
    except rospy.ROSInterruptException:
        pass
