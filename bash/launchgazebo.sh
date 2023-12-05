#! /usr/bin/bash

source ~/ros/devel/setup.bash
export HUSKY_LMS1XX_ENABLED=true
export HUSKY_LMS1XX_TOPIC=/front/scan/

roslaunch catkin_ic husky_geom.launch [verbose:=true]

