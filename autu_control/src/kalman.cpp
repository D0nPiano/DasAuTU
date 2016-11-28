#include "ros/ros.h"
#include "pses_basis/SensorData.h"
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core/mat.hpp"

typedef pses_basis::SensorData sensor_data;
using namespace cv;
// Mat: (Höhe, Breite)
double X = 0;
Mat CPos = (Mat_<double>(1,3) << 1, 0, 0);
Mat CVel = (Mat_<double>(1,3) << 0,1,0);
Mat x = (Mat_<double>(3,1) << 0,0,0);
//folgende Werte müssen noch eingstellt werden:
Mat P =(Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
Mat Q = (Mat_<double>(3,3) << 0,0,0,0,0.1, 0,0,0,0);
double u=0.1; //was ist das?
double R=0.2;
ros::Timer timer;
ros::Time startTimer;

//calc braucht etwa 38k ns auf einfachen Daten
void calc(const Mat_<double> C, const double y){
    //ros::Time  nowtime = ros::Time::now();
    Mat K = P*(C.t())/(C*P*(C.t())+R);
    x = (x + K*(y-C*x));
    //Mat::eye erstellt eine Einheitsmatrix der Größe 3x3 mit 64bit floating point (double) pro Feld
    P=(Mat::eye(3,3, CV_64F)-K*C)*P;
    //std::cout << (ros::Time::now()-nowtime).toNSec()<<  "calc done" << std::endl;
}
void sensorCallback(const sensor_data::ConstPtr& msg){
    timer.stop();

    //calc(CPos, gefahrene Distanz);
    calc(CVel, 1.0);
    timer.start();
    startTimer = ros::Time::now();
}
//interpolateCallback braucht etwa 6-25k ns, mit Schwankungen in Richtung 60k ns
void interpolateCallback(const ros::TimerEvent& t){
    double Calltime;
    ros::Time nowtime=ros::Time::now();
    if(t.last_real.toNSec()!=0)
        Calltime = (t.current_real-t.last_real).toNSec(); //Calltime = Time since last Call
    else
        Calltime = (t.current_real-startTimer).toNSec();
    //double sheduledCalltime = (t.current_expected-t.last_expected).toNSec();
    //std::cout << std::fixed << "real:  "<<Calltime << "  sheduled: " << sheduledCalltime << "  " << t.last_real.toNSec()<< "  " << t.current_real.toNSec() << std::endl; //fixed: verhindert wissenschaftliche Ausgabe des doubles (e^...)
    Mat A = (Mat_<double>(3,3) << 1, Calltime, Calltime*Calltime/2, 0, 1, Calltime, 0, 0, 1);
    Mat B = (Mat_<double>(3,1) << Calltime*Calltime/2, Calltime, 0);
    x=A*x+B*u;
    P=A*P*(A.t())+Q;
    std::cout << (ros::Time::now()-nowtime).toNSec()<<  "   interpolate done" << std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "kalman");
    ros::NodeHandle nh;
    //interpolateCallback wird etwa alle 110k ns aufgerufen, aber mit einigen Schwankungen in Richtung 140k ns und 50k ns
    timer = nh.createTimer(ros::Duration(0.001), std::bind(interpolateCallback));
    timer.stop();
    ros::Subscriber sub = nh.subscribe<sensor_data>("pses_basis/sensor_data", 10, sensorCallback);
    ros::spin();
    return 0;
}

//zu lösende Fragen: schwankende Rechenzeit bei interpolateCallback, ROS_INFO vs std::cout, Timer aufrufschwankungen und stop -> start Problematik


