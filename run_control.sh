#!/bin/sh

sudo su

source install/setup.bash
export ROS_DOMAIN_ID=12

ros2 run reflex-exo control_node