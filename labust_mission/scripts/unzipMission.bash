#!/bin/bash

echo "Unzipping mission archive... "
echo $1

unzip -o $1 -d /home/filip/catkin_ws/src/labust-ros-pkg/labust_mission/data/extracted/ 
echo "Archive unzipped."
