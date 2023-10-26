#include <ros/ros.h>

int main (int argc, char **argv){

    ros::init(argc, argv, "prova_nodo");

    ros::NodeHandle nh;

    ROS_INFO("nodo partito");

    ros::Duration(1.0).sleep();

    ROS_INFO("exit");


}