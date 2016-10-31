#Stuff you should remember
##Allgemein
User: pses
Password: letmein

Qhd verwenden

##Start des Autos
* roslaunch kinect2_bridge kinect2_bridge.launch
* Ps_basis mit sudo su ausführen:
* roslaunch pses_basis pses_basis kinect:=bool dashboard:=bool
* rqt (bzw. rosrun rqt_gui rqt_gui) -> zeigt kinect Bilder

##App Benutzung
ifconfig    -> Zum Auslesen der IP Adresse
Export ROS_IP="ipadresse" -> bei jeder node, auch kinect

##Neue Node anlegen
/catkin_ws/src/ catkin_create_pkg test_node roscpp ....


##Auto Betriebssystem Updaten
1. In src Ordner pullen (/catkin_ws/src/ git pull)
2. catkin_make ausführen in Ordner (/catkin_ws/)
