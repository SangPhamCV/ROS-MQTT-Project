#!/usr/bin/env python3

#THIS SCRIPT SEND THE GOAL PLAN OF ROBOT

import rospy
import actionlib
from std_msgs.msg import Float32MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math

class MoveBaseClientNode:
    def __init__(self):
    
        self.x_value = None
        self.y_value = None
        self.theta_value = None

        rospy.init_node('movebase_client_py')
        rospy.Subscriber('/goal_point', Float32MultiArray, self.callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def callback(self, data):

        received_list = data.data
        self.x_value, self.y_value, self.theta_value = received_list

    def movebase_client(self, value_x, value_y, value_theta):
        
        self.theta_radian = math.pi/180*value_theta
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
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def run(self):

        while not rospy.is_shutdown():
            if all(getattr(self, attr) is not None for attr in ['x_value', 'y_value', 'theta_value']):
                
                try:
                    run_request = input("Execute the path (OK)? ").lower()
                    if run_request == "ok":
                        result = self.movebase_client(self.x_value, self.y_value, self.theta_value)
                        if result:
                            rospy.loginfo("Goal execution done!")
                        else:
                            rospy.loginfo("Goal execution failed!")
                    else:
                        rospy.loginfo("Path execution canceled.")
                except rospy.ROSInterruptException:
                    rospy.loginfo("Navigation test finished.")

                self.x_value = None
                self.y_value = None
                self.theta_value = None

        
if __name__ == '__main__':

    move_base_node = MoveBaseClientNode()
    move_base_node.run()
