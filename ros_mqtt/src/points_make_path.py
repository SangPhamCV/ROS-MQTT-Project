#!/usr/bin/env python3

# THIS SCRIPT CALLS A SERVICE THAT HANDLES ALL THE POINTS, MAKING UP THE PATH 

import rospy
from ros_mqtt.srv import GetNavPath, GetNavPathRequest
from move_base_msgs.msg import MoveBaseActionGoal
import json
import time
from define_mqtt import DefineMqtt


define_mqtt = DefineMqtt()
define_mqtt.define()
topic_mqtt = rospy.get_param('topic_mqtt_path', "robot/point_path")


path_data = {}  # Initialize an empty dictionary

def call_path_service():
    rospy.wait_for_service('get_path')
    try:
        get_path = rospy.ServiceProxy('get_path', GetNavPath)
        response = get_path(GetNavPathRequest())
        time.sleep(0.5)

        poses = response.get_path.poses
        positions = [pose.pose.position for pose in poses]
        path_data["positions"] = [{"x": round(p.x, 2), "y": round(p.y, 2)} for p in positions]
        print(len(positions))
        
        path_data_json = json.dumps(path_data)
        define_mqtt.mqtt_client.publish(topic_mqtt, str(path_data_json), qos=0)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def goal_pose_callback(msg):
    goal_pose = msg.goal.target_pose.pose
    _ = goal_pose.position
    call_path_service()

if __name__ == '__main__':
    rospy.init_node('call_service_handle')
    rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, goal_pose_callback)

    try:
        define_mqtt.mqtt_client.loop_start()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        define_mqtt.mqtt_client.disconnect()    
    
    
