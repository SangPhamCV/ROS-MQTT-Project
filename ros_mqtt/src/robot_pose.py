#!/usr/bin/env python3

# THIS SCRIPT SEND THE POSE OF ROBOT TO MQTT SERVER

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
from tf.transformations import euler_from_quaternion
from define_mqtt import DefineMqtt

define_mqtt = DefineMqtt()

define_mqtt.define()

topic_mqtt = rospy.get_param('topic_mqtt_pose', "robot/pose")

def amcl_pose_callback(msg):
    pose = msg.pose.pose
    orientation_quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(orientation_quaternion)
    
    pose_data = {
        "pose_position": {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z
        },
        "orientation": {
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw
        }
    }
    pose_data_json = json.dumps(pose_data) 
    define_mqtt.mqtt_client.publish(topic_mqtt, pose_data_json, qos=0)  

def main():
    rospy.init_node('amcl_pose', anonymous=True)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)

    try:
        define_mqtt.mqtt_client.loop_start()  
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        define_mqtt.mqtt_client.disconnect()

if __name__ == '__main__':
    main()
