#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray, Float32
from threading import Thread, Event
import paho.mqtt.client as mqtt
from define_mqtt import DefineMqtt
import json

define = DefineMqtt()
define.define()

error_pub_mqtt = rospy.get_param('error_pub_mqtts', "robot/error")

camera_front = camera_behind = False
lidar = False
servo_1 = servo_2 = servo_3 = 0
ai = navigation = location = None
a = e = f = g = h = q = None
b = c = d = 0
data_sent = data_sent1 = False

def camera_status_callback(data):
    global camera_front, camera_behind
    if len(data.data) == 2:
        camera_1, camera_2 = data.data
        camera_front = bool(camera_1)
        camera_behind = bool(camera_2)
        rospy.loginfo("Received camera_front: %d, camera_behind: %d", camera_front, camera_behind)
    

def servo_status_callback(data):
    global servo_1, servo_2, servo_3
    if len(data.data) == 3:
        servo_1, servo_2, servo_3 = data.data
        rospy.loginfo("Received servo_1: %d, servo_2: %d, servo_3: %d", servo_1, servo_2, servo_3)
       

def lidar_status_callback(data):
    global lidar
    lidar_float = data.data
    lidar = bool(lidar_float)
    print(lidar)

def ai_status_callback(data):
    global ai
    pass

def navi_status_callback(data):
    global navigation
    pass

def location_status_callback(data):
    global location
    pass

def hehe():
    global camera_front, camera_behind, servo_1, servo_2, servo_3, lidar, ai
    global navigation, location, a, b, c, d, e, f, g, h, q

    if(lidar != a or servo_1 != b or servo_2 != c or servo_3 != d or camera_front != e or camera_behind != f):    
        print(lidar, servo_1, servo_2, servo_3, camera_front, camera_behind)

        data = {
            "lidar_status": lidar,
            "servo1_status": int(servo_1),
            "servo2_status": int(servo_2),
            "servo3_status": int(servo_3),
            "camera_d435i_front": camera_front,
            "camera_d435i_behind": camera_behind,
            "ai_status": False,
            "navigation_status": False,
            "location_status": False
        }
        
        json_data = json.dumps(data)
        define.mqtt_client.publish(error_pub_mqtt, json_data)

        a = lidar
        b = servo_1
        c = servo_2
        d = servo_3
        e = camera_front
        f = camera_behind

def pub_error(pub):
    global camera_front, camera_behind, servo_1, servo_2, servo_3, lidar, ai
    global navigation, location, data_sent, data_sent1


    if((camera_front == True or camera_behind == True or lidar == True or servo_1 != 0 or servo_2 != 0 or servo_3 != 0) and not data_sent):
        error = 1
        pub.publish(error)
        data_sent = True
        data_sent1 = False
    
    elif(camera_front == camera_behind == lidar == False and servo_1 == servo_2 == servo_3 == 0 and not data_sent1):
        error = 0
        pub.publish(error)
        data_sent = False
        data_sent1 = True


def subscriber():
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber('camera_status', Int32MultiArray, camera_status_callback)
    rospy.Subscriber('alarm_pub', Int32MultiArray, servo_status_callback)
    rospy.Subscriber('lidar_status', Float32, lidar_status_callback)
    pub = rospy.Publisher('error', Float32, queue_size=10, latch=True)
    while not rospy.is_shutdown():
        hehe()
        pub_error(pub)

    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
