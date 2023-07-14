#! /usr/bin/bash

source ~/ros/devel/setup.bash
gnome-terminal -e "bash launchgazebo.sh"
sleep 10
gnome-terminal -e "bash launchrviz.sh"
gnome-terminal -e "bash amcltest.sh"

cd ~/ros/src/rosbag/
rosbag play teste2.bag
cd


