# ROS-MQTT Project

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)

## Overview

This package implements ros-mqtt communication.

## Features

- Feature 1: Subscribe to the data goal plan and control commands for the robot.
- Feature 2: Publish the robot's position and all the points that make up the path from the robot's current position to the goal plan.
- Feature 3: Calculate the estimated/actual distance.
- Feature 4: Generate a path connecting any two points.
- Feature 5: Publish sensor data to MQTTT.

## Installation

```bash
$ cd ~/your-workspace/src
$ git clone https://github.com/SangPhamCV/ROS-MQTT-Project.git
$ cd ..
$ catkin_make
```

## Usage

```bash
$ roslaunch ros_mqtt mqtt.launch
