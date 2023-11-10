#!/usr/bin/env python3

import rospy
import paho.mqtt.client as mqtt

class DefineMqtt:
    def __init__(self):
        self.mqtt_client = mqtt.Client()

    def define(self):
        self.mqtt_broker_host = rospy.get_param('broker_address', "10.14.7.14")
        self.mqtt_broker_port = rospy.get_param('broker_port', 1883)
        self.username = rospy.get_param('username', "hehe")
        self.password = rospy.get_param('password', "hihi")

        self.mqtt_client.username_pw_set(username=self.username, password=self.password)
        self.mqtt_client.connect(self.mqtt_broker_host, self.mqtt_broker_port)
