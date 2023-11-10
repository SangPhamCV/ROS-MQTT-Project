#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseWithCovarianceStamped

goal_plan = [0, -2]
pose = None  # Define pose as a global variable

def amcl_callback(msg):
    global pose
    pose = msg.pose.pose.position
def movebase_client():
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

def get_global_plan(start, goal):
    # Create a service client to the global planner
    plan_service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)

    try:
        plan = plan_service(start, goal, 0.0)
        waypoints = plan.plan.poses
        for i, waypoint in enumerate(waypoints):
            print(f"Waypoint {i + 1}: x={waypoint.pose.position.x}, y={waypoint.pose.position.y}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('navigation_client')
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_callback)

    # Wait for the pose information to be available
    while pose is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    # Define the start and goal poses for planning
    start = PoseStamped()
    start.header.frame_id = "map"
    
    # Check if pose is not None before accessing its attributes
    if pose is not None:
        start.pose.position = Point(x=pose.x, y=pose.y)

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position = Point(x=goal_plan[0], y=goal_plan[1])

    try:
        get_global_plan(start, goal)
        ahah = input("OK? ").lower()
        if ahah == "ok":
            movebase_client()
        else:
            pass
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished")
