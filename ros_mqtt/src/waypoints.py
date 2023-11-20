#!/usr/bin/env python3

import rospy
import json
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan
from std_msgs.msg import Float32MultiArray
from define_mqtt import DefineMqtt
from tf.transformations import euler_from_quaternion


# Define the MQTT broker and topics
mqtt = DefineMqtt()
mqtt.define()

mqtt_sub_points = rospy.get_param('mqtt_sub_points', "robot/haha")
mqtt_pub_waypoints = rospy.get_param('mqtt_pub_waypoints', "robot/bleble")
mqtt_pub_pose = rospy.get_param('mqtt_pub_pose', "robot/qwerty")

current_position = []

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo("Connected to MQTT broker")
        mqtt.mqtt_client.subscribe(mqtt_sub_points)
    else:
        rospy.logwarn("Connection failed")

def amcl_callback(data):
    global current_position
    pose = data.pose.pose
    current_position = [pose.position.x, pose.position.y]
    orientation_quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(orientation_quaternion)
    
    pose_data = {
        "pose_position": {
            "x": round(pose.position.x, 3),
            "y": round(pose.position.y, 3),
            "z": round(pose.position.z, 3)
        },
        "orientation": {
            "roll": round(roll, 3),
            "pitch": round(pitch, 3),
            "yaw": round(yaw, 3)
        }
    }
    pose_data_json = json.dumps(pose_data) 
    mqtt.mqtt_client.publish(mqtt_pub_pose, pose_data_json)
# the function generate a path connecting two points
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
        waypoints_list = []
        for i, waypoint in enumerate(waypoints):
            waypoint_info = {
                "x": round(waypoint.pose.position.x, 3),
                "y": round(waypoint.pose.position.y, 3)
            }
            waypoints_list.append(waypoint_info)
        return waypoints_list
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
            mqtt.mqtt_client.publish(mqtt_pub_waypoints, json.dumps(waypoints))
            goal_msg = Float32MultiArray(data=goal_data)  # Initialize with data
            publish_goal.publish(goal_msg)

    elif goal_data == [9999, 9999, 9999] and current_position is not None:
        goal_data = current_position
        waypoints = plan_path(start_data, goal_data)
        if waypoints:
            mqtt.mqtt_client.publish(mqtt_pub_waypoints, json.dumps(waypoints))
    else:
        waypoints = plan_path(start_data, goal_data)
        if waypoints:
            mqtt.mqtt_client.publish(mqtt_pub_waypoints, json.dumps(waypoints))

if __name__ == '__main__':
    rospy.init_node('test_multi_topic')
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_callback)
    publish_goal = rospy.Publisher('/goal_point', Float32MultiArray, queue_size=10)

    mqtt.mqtt_client.on_connect = on_connect
    mqtt.mqtt_client.on_message = on_message

    mqtt.mqtt_client.loop_start()
    rospy.spin()
