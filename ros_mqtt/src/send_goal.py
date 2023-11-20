#!/usr/bin/env python3

import rospy
import actionlib
from std_msgs.msg import Float32MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
from define_mqtt import DefineMqtt  # Import the DefineMqtt module

class MoveBaseClientNode:
    def __init__(self):
        self.x_value = None
        self.y_value = None
        self.theta_value = None
        self.run_request = None

        rospy.init_node('movebase_client_py')
        rospy.Subscriber('/goal_point', Float32MultiArray, self.callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # Create an instance of DefineMqtt
        self.define = DefineMqtt()
        
        self.mqtt_sub_move = rospy.get_param('mqtt_sub_move', 'robot/eheheh')

        self.define.define()
        self.define.mqtt_client.on_message = self.on_mqtt_message
        self.define.mqtt_client.subscribe(self.mqtt_sub_move)
        self.define.mqtt_client.loop_start()

    def callback(self, data):
        received_list = data.data
        self.x_value, self.y_value, self.theta_value = received_list

    def on_mqtt_message(self, client, userdata, msg):
        self.run_request = msg.payload.decode()

    def movebase_client(self, value_x, value_y, value_theta):
        self.theta_radian = math.pi/180 * value_theta
        self.z_value = math.sin(self.theta_radian/2)
        self.w_value = math.cos(self.theta_radian/2)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = value_x
        goal.target_pose.pose.position.y = value_y
        goal.target_pose.pose.orientation.z = self.z_value
        goal.target_pose.pose.orientation.w = self.w_value

        self.client.send_goal(goal)

        rospy.loginfo("Executing goal...")
        while not rospy.is_shutdown():
            if self.run_request.lower() == "cancel":
                self.client.cancel_goal()
                rospy.loginfo("Goal execution canceled.")
                self.run_request = None
                break

            if self.client.get_state() in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED]:
                rospy.loginfo("Goal execution done!")
                break

    def run(self):
        while not rospy.is_shutdown():
            if all(getattr(self, attr) is not None for attr in ['x_value', 'y_value', 'theta_value']):
                try:
                    self.run_request = None
                    print("Execute the path (start)? ")
                    while self.run_request is None:
                        rospy.sleep(0.1)
                    if self.run_request.lower() == "start":
                        result = self.movebase_client(self.x_value, self.y_value, self.theta_value)
                    else:
                        rospy.loginfo("Path execution canceled.")

                    self.run_request = None
                except rospy.ROSInterruptException:
                    rospy.loginfo("Navigation test finished.")

                self.x_value = None
                self.y_value = None
                self.theta_value = None

if __name__ == '__main__':
    move_base_node = MoveBaseClientNode()
    move_base_node.run()
