#!/bin/bash
roslaunch moos_ros_3d.launch &
rosrun moosros Bridge uuv_msgs.xml uuv.moos __ns:=uuv __name:=UUVBridge&
rosrun moosros Bridge usv_msgs.xml usv.moos __ns:=usv __name:=USVBridge&
rosrun UWSim UWSim --configfile test_scene.xml
