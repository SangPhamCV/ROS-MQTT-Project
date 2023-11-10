#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import rospy
import json
from geometry_msgs.msg import PoseStamped, Quaternion, Point, PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray


# Define the MQTT broker and topics
mqtt_broker = "10.14.7.175"
mqtt_port = 1883
username = 'mascot'
password = 'demo'
subscribe_topic = "robot/request_direction"
publish_topic = "robot/pose"

client = mqtt.Client()

current_position = []

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo("Connected to MQTT broker")
        client.subscribe(subscribe_topic)
    else:
        rospy.logwarn("Connection failed")

def amcl_callback(data):
    global current_position
    pose = data.pose.pose
    current_position = [pose.position.x, pose.position.y]

def plan_path(start, goal):
    try:
        rospy.wait_for_service('/move_base/NavfnROS/make_plan')
        plan_service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)
    except rospy.ROSException as e:
        print(f"Failed to connect to the service: {e}")
        return

    start_pose = PoseStamped()
    start_pose.header.frame_id = "map"
    start_pose.pose.position = Point(x=start[0], y=start[1])

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position = Point(x=goal[0], y=goal[1])

    try:
        plan = plan_service(start_pose, goal_pose, 0.0)
        waypoints = plan.plan.poses
        waypoints_dict = {}  
        for i, waypoint in enumerate(waypoints):
            waypoint_info = {
                "x": waypoint.pose.position.x,
                "y": waypoint.pose.position.y
            }
            waypoints_dict[i] = waypoint_info  
        return waypoints_dict
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode('utf-8'))
    start_data = data.get('start')
    goal_data = data.get('goal')
    if start_data == [9999, 9999, 9999] and current_position is not None:
        start_data = current_position
        waypoints = plan_path(start_data, goal_data)
        if waypoints:
            client.publish(publish_topic, json.dumps(waypoints))
            goal_msg = Float32MultiArray(data=goal_data)  # Initialize with data
            publish_goal.publish(goal_msg)

    elif goal_data == [9999, 9999, 9999] and current_position is not None:
        goal_data = current_position
        waypoints = plan_path(start_data, goal_data)
        if waypoints:
            client.publish(publish_topic, json.dumps(waypoints))
    else:
        waypoints = plan_path(start_data, goal_data)
        if waypoints:
            client.publish(publish_topic, json.dumps(waypoints))
    print(json.dumps(waypoints))
if __name__ == '__main__':
    rospy.init_node('test_multi_topic')
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_callback)
    publish_goal = rospy.Publisher('/goal_point', Float32MultiArray, queue_size=10)

    client.on_connect = on_connect
    client.on_message = on_message

    client.username_pw_set(username, password)
    client.connect(mqtt_broker, mqtt_port, 60)

    client.loop_start()
    rospy.spin()

