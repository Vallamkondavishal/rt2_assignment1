#!/bin/bash

gnome-terminal --tab --title="ros" -- bash -c "source ros.sh; roslaunch rt2_assignment1_ros1 sim.launch"
gnome-terminal --tab --title="bridge" -- bash -c "sleep 1; source ros12.sh; ros2 run ros1_bridge dynamic_bridge"
gnome-terminal --tab --title="container" -- bash -c "source ros2.sh; ros2 launch rt2_assignment1_ros2 sim_launch.py"
