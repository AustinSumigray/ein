#!/bin/bash

# This file, when sourced, sets your ROS environment variables.  It
# should be placed in the root of the catkin workspace, since it also
# sources the catkin setup.bash file.  It takes a ROBOT= enviornment
# variable which it uses to set things.  You may need to edit this
# file depending on your network configuration to set ROS_IP or
# ROS_HOSTNAME for your machine. 

if [ -z ${ROBOT} ]; then
export ROBOT=localhost
fi 

source devel/setup.bash
#export ROS_IP=192.168.42.1
export ROS_HOSTNAME=`hostname`
export ROS_MASTER_URI=http://$ROBOT:11311
export PS1="\[\033[00;33m\][pidrone - ${ROS_MASTER_URI}]\[\033[00m\] $PS1"

# If you are using baxter, make this file source baxter.sh and remove
# the othe rstuff above, which is redundant if you are using
# baxter.sh.  

# source baxter.sh
