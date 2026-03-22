#!/bin/bash
export ROS_DOMAIN_ID=30
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
source /opt/ros/jazzy/setup.bash
source /home/rpi4/ros2_ws/install/setup.bash
exec ros2 run agv_test_pkg web_sensor_dashboard
