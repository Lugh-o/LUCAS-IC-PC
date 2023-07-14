#! /usr/bin/bash

source ~/ros/devel/setup.bash
gnome-terminal -e "bash launchgazebo.sh"
sleep 5
gnome-terminal -e "bash launchrviz.sh"
sleep 2
gnome-terminal -e "bash amcl_geom.sh"
sleep 5


