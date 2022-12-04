#!/bin/bash

source /home/zwh/zwh_main/devel/setup.bash

gnome-terminal --window -e "roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=192.168.56.101"
sleep 10s

gnome-terminal --window -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true"
sleep 2s

gnome-terminal --window -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true"
