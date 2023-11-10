#!/usr/bin/env python3

import rospy
import actionlib
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Quaternion, Point, PoseWithCovarianceStamped
import json
import paho.mqtt.client as mqtt
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion

broker_address = rospy.get_param('broker_address', "10.14.7.175")
broker_port = rospy.get_param('broker_port', 1883)
username = rospy.get_param('username', "mascot")
password = rospy.get_param('password', "demo")

sub_request = rospy.get_param('sub_request', "robot/request_direction")
pub_waypoints = rospy.get_param('pub_waypoints', 'robot/waypoints')
pub_robotPose = rospy.get_param('pub_robotPose', 'robot/pose')

current_position = []
goal_plan = []

client = mqtt.Client()

def amcl_callback(data):
    global current_position
    pose = data.pose.pose
    current_position = [pose.position.x, pose.position.y]

    # orientation_quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    # roll, pitch, yaw = euler_from_quaternion(orientation_quaternion)
    
    # pose_data = {
    #     "pose_position": {
    #         "x": pose.position.x,
    #         "y": pose.position.y,
    #         "z": pose.position.z
    #     },
    #     "orientation": {
    #         "roll": roll,   
    #         "pitch": pitch,
    #         "yaw": yaw
    #     }
    # }
    # pose_data_json = json.dumps(pose_data) 
    # client.publish(pub_robotPose, pose_data_json)

# Function to plan the path between the start and goal
def plan_path(start, goal):
    try:
        rospy.wait_for_service('/move_base/NavfnROS/make_plan')
        plan_service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)
    except rospy.ROSException as e:
        print(f"Failed to connect to the service: {e}")
        return

    # Define the start and goal poses for planning
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

        waypoints_json = json.dumps(waypoints_dict)
        ahaha = waypoints_json
        client.publish(pub_waypoints, ahaha)
        waypoints_dict.clear()

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

# Callback when a message is received on the MQTT topic
def on_message(client, userdata, message):
    data = json.loads(message.payload.decode())
    start_data = data.get('start')
    goal_data = data.get('goal')

    if start_data == [9999, 9999] and current_position is not None:
        start_data = current_position
        plan_path(start_data, goal_data)  # Call plan_path with the received data
        run_request = input("Execute the path (OK)? ").lower()
        if run_request == "ok":
            movebase_client(goal_data)
        else:
            rospy.loginfo("Path execution canceled.")
    elif goal_data == [9999, 9999] and current_position is not None:
        goal_data = current_position
        plan_path(start_data, goal_data)
    else:
        plan_path(start_data, goal_data)

def movebase_client(goal_plan):
    # Create a SimpleActionClient
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Create a MoveBaseGoal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Check if pose is not None before accessing its attributes
    goal.target_pose.pose.position = Point(x=goal_plan[0], y=goal_plan[1])
    goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

    # Send the goal and wait for the result
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()

    if result:
        rospy.loginfo("Goal execution done!")
    else:
        rospy.logerr("Action server did not return a result!")

if __name__ == '__main__':
    rospy.init_node('waypoint_publisher')
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_callback)
    # rospy.Publisher('publist_dict', )
    client.username_pw_set(username, password)
    client.connect(broker_address, port=broker_port)

    client.subscribe(sub_request)  # Subscribe to the specified MQTT topic

    client.on_message = on_message  # Set the on_message callback

    client.loop_start()
    
    # You need to start the rospy spinner to keep the node running
    rospy.spin()
