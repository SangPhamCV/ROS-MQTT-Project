#!/usr/bin/env python3

# THIS SCRIPT SUBSCRIBE TOPIC "/move_base/NavfnROS/plan"
# AND MAKE THE DATA HANDLE ROS SERVICE SERVER

import rospy
from ros_mqtt.srv import GetNavPath, GetNavPathResponse
from nav_msgs.msg import Path
import time

path_data = None

def path_callback(data):
    global path_data
    path_data = data

def handle_get_path(req):
    if path_data is not None:
        time.sleep(0.5)
        return GetNavPathResponse(path_data)
    else:
        return GetNavPathResponse(None)

def subscriber():
    rospy.init_node('get_handle_global_path', anonymous=True)

    rospy.Subscriber('/move_base/NavfnROS/plan', Path, path_callback)
    path_service = rospy.Service('get_path', GetNavPath, handle_get_path)

    rospy.spin()

if __name__ == '__main__':
    subscriber()
