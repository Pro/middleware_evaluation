# !/bin/bash
MASTER=raspberrypi
LOCAL=$(hostname -I | awk '{print $1}')

source /opt/ros/kinetic/setup.bash
source $HOME/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://$MASTER:11311/
export ROS_IP=$LOCAL

rosrun beginner_tutorials testing_client
