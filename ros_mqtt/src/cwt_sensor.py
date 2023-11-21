#!/usr/bin/env python3

import rospy
import json
from pymodbus.client.sync import ModbusSerialClient
import paho.mqtt.client as mqtt
from datetime import datetime
import time
from define_mqtt import DefineMqtt

define = DefineMqtt()
define.define()

cwt_pub_mqtt = rospy.get_param('cwt_pub_mqtts', "env/cwt_sensor")

client_modbus = ModbusSerialClient(method='rtu', port='/dev/ttyUSB0', baudrate=9600, stopbits=1, bytesize=8, parity='N', timeout=1)

def current_time():
    now = datetime.now().isoformat()
    return now

def read_sensor():
    if client_modbus.connect():
        result = client_modbus.read_holding_registers(address=0, count=6, unit=1)
        co2 = result.registers[0]
        pm25 = result.registers[3]
        humidity = result.registers[4]
        temperature = result.registers[5]

        if result.isError():
            print("Register Errror")
        else:
            data = {
                "co2": str(co2),
                "pm25": str(pm25),
                "temp": str(temperature/100),
                "humidity": str(humidity/100),
                "dateTime": current_time()
                }

            json_data = json.dumps(data, ensure_ascii=False).encode('utf-8').decode()
            define.mqtt_client.publish(cwt_pub_mqtt, json_data)
    else:
        print("Modbus connected faild")

def main():
    rospy.init_node('cwt_sensor')
    rate = rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            read_sensor()
            define.mqtt_client.loop_start()
            rate.sleep()
    except KeyboardInterrupt:
        define.mqtt_client.disconnect()

if __name__ == "__main__":
    main()
