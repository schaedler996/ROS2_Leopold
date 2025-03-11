#!/bin/bash
echo "Build started"
colcon build 

echo "Build finished"
source ~/ros2_ws/install/setup.bash
echo "Sourced"