#!/usr/bin/env python3

# THIS SCRIPT CALCULATE THE ESTAMATE/ ACTUAL DISTANCE OF GLOBAL PLANNER

import numpy as np
import rospy
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Pose
from copy import deepcopy
from define_mqtt import DefineMqtt
import json

distance_array = []
previous_pose = None
current_goal = None 

define_mqtt = DefineMqtt()

define_mqtt.define()

topic_mqtt = rospy.get_param('topic_mqtt_cal', "robot/cal")

def callback(path_msg):
    global path_length
    path_length = 0

    for i in range(len(path_msg.poses) - 1):    # calculate distance fomular
        position_a_x = path_msg.poses[i].pose.position.x
        position_b_x = path_msg.poses[i+1].pose.position.x
        position_a_y = path_msg.poses[i].pose.position.y
        position_b_y = path_msg.poses[i+1].pose.position.y
        path_length += np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y - position_a_y), 2))
    distance = round(path_length, 2)
    distance_array.append(distance)

    actual_distance = distance_array[-1]
    estimate_distance = distance_array[0]
    time_estimation = estimate_distance / 0.8 # 0.8 is avg velocity of robot
    # when the robot's pose has changed, clead the path_distance and recalculated
    if goal_pose_callback() is False:
        distance_array.clear()
    
    path_distance = {
        "estimate_distance": estimate_distance,
        "time_estimation": time_estimation,
        "actual_distance": actual_distance
    }
    path_distance_json = json.dumps(path_distance)
    define_mqtt.mqtt_client.publish(topic_mqtt, path_distance_json, qos=0)

# The function knows when the robot's pose has changed
def goal_pose_callback():
    global previous_pose
    if previous_pose is None:
        previous_pose = Pose()        
        return True 

    current_pose = current_goal.goal.target_pose.pose
    if previous_pose is None:
        previous_pose = deepcopy(current_pose)
        return True 

    if current_pose != previous_pose:
        previous_pose = deepcopy(current_pose)
        return False 

def current_goal_callback(msg):
    global current_goal
    current_goal = msg

def main():
    rospy.init_node('cal_distance')
    rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, current_goal_callback)
    rospy.Subscriber('/move_base/NavfnROS/plan', Path, callback)

    try:
        define_mqtt.mqtt_client.loop_start()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        define_mqtt.mqtt_client.disconnect()

if __name__ == '__main__':
    print("start")
    main()
