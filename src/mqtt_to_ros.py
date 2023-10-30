#!/usr/bin/env python3

# THIS SCRIPT SUBSCRIBE MQTT DATA

import rospy
import paho.mqtt.client as mqtt
import json
from std_msgs.msg import Float64MultiArray

broker_address = rospy.get_param('broker_address', "10.14.10.175")
broker_port = rospy.get_param('broker_port', 1884)
username = rospy.get_param('username', "hihi")
password = rospy.get_param('password', "hehe")

topic_mqtt = rospy.get_param('topic_mqtt_sub', "robot/list_data")

client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo("Connected to MQTT broker")
        client.subscribe(topic_mqtt)
    else:
        rospy.logwarn("Connection failed")

def on_message(client, userdata, message):
    data = json.loads(message.payload.decode())
    rospy.loginfo("Received list: %s", data)
    ros_msg = Float64MultiArray(data=data)
    pub.publish(ros_msg)
    
if __name__ == '__main__':
    rospy.init_node('mqtt_subscriber')
    pub = rospy.Publisher('ros_topic', Float64MultiArray, queue_size=10)

    client.on_connect = on_connect
    client.on_message = on_message

    client.username_pw_set(username, password)
    client.connect(broker_address, port=broker_port)

    client.loop_start()
    rospy.spin()
