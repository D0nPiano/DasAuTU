#!/bin/bash

ipaddress="$(hostname -I | sed 's/\s.*$//')"

export ROS_IP=$ipaddress
export ROS_HOSTNAME="$ipaddress"
export ROS_MASTER_URI="http://10.42.0.1:11311/"
echo Schönen Tag noch Nummer $ipaddress!