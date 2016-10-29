#!/bin/bash
user=pses

read -p "Skript kann durchaus 1 Stunde benötigen. Fortfahren? (y / n): " -n 1 -r
echo    # (optional) move to a new line
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    exit 1
fi

sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get -y install chromium-browser
sudo apt-get -y install gedit
sudo add-apt-repository -y ppa:webupd8team/sublime-text-3
sudo apt-get update
sudo apt-get -y install sublime-text-installer
sudo apt-get -y install cutecom
sudo apt-get -y install git
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get -y install ros-indigo-desktop-full
sudo rosdep init
rosdep update
sudo apt-get -y install python-rosinstall
mkdir -p /home/$user/catkin_ws/src
sudo apt-get -y install --reinstall cmake
echo "source /opt/ros/indigo/setup.bash" >> /home/pses/.bashrc
source /home/$user/.bashrc

cd /home/$user/catkin_ws/src
catkin_init_workspace
cd /home/$user/catkin_ws/
catkin_make
sudo su <<EOSU
echo "source /opt/ros/indigo/setup.bash" >> /etc/bash.bashrc
echo "source /home/$user/catkin_ws/devel/setup.bash" >> /etc/bash.bashrc
EOSU
source  /etc/bash.bashrc

sudo apt-get -y install qtbase5-dev
cd /home/$user/catkin_ws/src
rm CMakeLists.txt
git clone https://github.com/tud-pses/PSES-Basis.git .
cd ..
rosdep install -r --from-paths .
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