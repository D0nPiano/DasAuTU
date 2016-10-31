#!/bin/bash
user=pses



cd /home/$user/catkin_ws/src
catkin_init_workspace
cd /home/$user/catkin_ws/
catkin_make