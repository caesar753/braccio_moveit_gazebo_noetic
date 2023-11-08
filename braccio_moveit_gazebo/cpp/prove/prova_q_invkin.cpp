#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
// #include <../InvKin.hpp>
#include "auto_targetter_iperreduced.hpp"
// Arm3Link InvKin;



int main(int argc, char** argv) {

    ros::init(argc, argv, "prova_q_invkin");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spin(1);
    spin.start();

    BraccioObjectInterface auto_targ;

    // auto_targ.loadCalibrate();
    
    std::vector<double> s_vec;
    double s = 2.30;
    s_vec[0] = s;
    std::vector<double> q = InvKin.inv_kin(s, Z_MIN, Z_MAX_DOWN, -M_PI / 2);
    // std::vector<double> q = InvKin.optimizeIK(s, Z_MIN, Z_MAX_DOWN, -M_PI / 2);
    return 0;



}