#!/bin/bash
user=pses

sudo apt-get -y install qtbase5-dev
cd /home/$user/catkin_ws/src
rm CMakeLists.txt
git clone https://github.com/tud-pses/PSES-Basis.git .
cd ..
rosdep install -r --from-paths .
sudo apt-get -y install ros-indigo-serial
catkin_make
cd ..
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
cd depends; ./download_debs_trusty.sh
sudo apt-get -y install build-essential cmake pkg-config
sudo dpkg -i debs/libusb*deb
sudo apt-get -y install libturbojpeg libjpeg-turbo8-dev
sudo dpkg -i debs/libglfw3*deb
sudo apt-add-repository -y ppa:floe/beignet
sudo apt-get update 
sudo apt-get -y install beignet-dev 
sudo dpkg -i debs/ocl-icd*deb
sudo dpkg -i debs/{libva,i965}*deb
sudo apt-get -y install -f
sudo apt-add-repository -y ppa:deb-rob/ros-trusty && sudo apt-get update
cd ..
mkdir build && cd build
cmake .. -DENABLE_CXX11=ON -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
sudo make install
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
cd /home/$user/catkin_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd /home/$user/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
#sudo cp /home/$user/org.freedesktop.upower.policy /usr/share/polkit-1/actions/