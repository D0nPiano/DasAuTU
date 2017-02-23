#!/bin/bash

mkdir -p ~/bag/rundkurs

cd ~/bag/rundkurs

rosbag record --duration=5m tf scan autu/rundkurs/corner autu/rundkurs/info