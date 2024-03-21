source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12

if [ $# -eq 1 ]; then
	ros2 run finex sensor_node.py "$1"
else
	ros2 run finex sensor_node.py
fi

