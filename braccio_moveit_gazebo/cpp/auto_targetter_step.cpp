#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
// #include <jsoncpp/json/json.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <custom_msgs/for_cpp.h>
#include <custom_msgs/matrix.h>

#include <nlohmann/json.hpp>



#include "InvKin.hpp"

// Constants
const double THETA_EXT = 0.27;
const double THETA_RET = M_PI / 4;
const double L_FUDGE = 0.08;
const double Z_MAX_SIDE = -0.03;
const double Z_MAX_DOWN = 0;
const double Z_MIN = -0.045;
const double CLOSE_ENOUGH = 0.02;
const double DEFAULT_ROT = 0;
const double S_SIDE_MAX = 0.4;
const double S_SIDE_MIN = 0.161;
const double S_TOP_MAX = 0.29;

Arm3Link InvKin;

using json = nlohmann::json;

//function which transforms cartesian coord into polar coord
std::pair<double, double> cart2pol(double x, double y) {
  // helper, convert cartesian to polar coordinates
  double rho = std::sqrt(x * x + y * y);
  double phi = std::atan2(y, x);
  return {rho, phi};
}

//function which transforms polar coord into cartesian
std::pair<double, double> pol2cart(double rho, double phi) {
  // helper, convert polar to cartesian coordinates
  double x = rho * std::cos(phi);
  double y = rho * std::sin(phi);
  return std::make_pair(x, y);
}

//function that, getting the angle of the shoulder, returns the angles of the wrist and of the elbow
std::pair<double, double> get_other_angles(double theta_shoulder) {
  // helper, converting some angles
  double theta_wrist = theta_shoulder + M_PI / 2;
  double theta_elbow = M_PI / 2 - 2 * theta_shoulder;
  return std::make_pair(theta_wrist, theta_elbow);
}

class BraccioObjectInterface{
    
    private:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface arm_group_;
    moveit::planning_interface::MoveGroupInterface gripper;
    cv::Mat homography_;

    public:
    BraccioObjectInterface() : nh_("~"), arm_group_("braccio_arm"), gripper("braccio_gripper"){
        moveit::planning_interface::MoveGroupInterface::Options arm_options("braccio_arm", "robot_description", nh_);
        moveit::planning_interface::MoveGroupInterface::Options gripper_options("braccio_gripper", "robot_description", nh_);
        // arm_group_ = moveit::planning_interface::MoveGroupInterface(arm_options);
        // gripper = moveit::planning_interface::MoveGroupInterface(gripper_options);
    }

    void callback_received_sherds(const custom_msgs::for_cpp& msg){

        ROS_INFO("Message_received; %s, %s", msg.sherds, msg.homes);
    }



    // void calibrate() {
    //     std::vector<std::vector<double>> src_pts;
    //     std::vector<std::vector<double>> dst_angs;

    //     // Replace with your actual implementation for getting link positions
    //     double mouseX, mouseY, r_;
    //     get_link_position(std::vector<std::string>{"kuka::base_link"});
    //     src_pts.push_back({mouseX, mouseY});

    //     gripperMiddle();
    //     int N = 8;
    //     double phi_min = M_PI / 6;
    //     double phi_max = M_PI - M_PI / 6;

    //     for (int i = 2; i < N; ++i) {
    //         goRaise();
    //         if (i % 2 == 0) {
    //             double rand_phi = phi_min + i * (phi_max - phi_min) / N;
    //             double theta_shoulder = THETA_RET; // Define THETA_RET
    //         } else {
    //             double theta_shoulder = THETA_EXT; // Define THETA_EXT
    //             double theta_wrist, theta_elbow;
    //             get_other_angles(theta_shoulder, theta_elbow, theta_wrist);
    //             std::vector<double> rand_targ = {rand_phi, theta_shoulder, theta_elbow, theta_wrist};
    //             goJoint(rand_phi, theta_shoulder, theta_elbow, theta_wrist);

    //             // Replace with your actual implementation for getting link positions
    //             get_link_position(std::vector<std::string>{"kuka::left_gripper_link", "kuka::right_gripper_link"});
    //             src_pts.push_back({mouseX, mouseY});
    //             dst_angs.push_back(rand_targ);
    //         }
    //     }

    //     std::ofstream file("calibration.json");
    //     if (file.is_open()) {
    //         json calibration_data = {{"src_pts", src_pts}, {"dst_angs", dst_angs}};
    //         file << calibration_data.dump(4);
    //         file.close();

    //         load_calibrate();
    //         goUp();
    //     } else {
    //         std::cerr << "Failed to open calibration.json for writing" << std::endl;
    //     }
    // }

    void loadCalibrate() {
    try {
        std::ifstream file("calibration.json");
        if (!file.is_open()) {
            std::cerr << "calibration.json not in current directory, run calibration first" << std::endl;
            return;
        }
        json calib;
        file >> calib;
        file.close();

        std::vector<std::vector<double>> src_pts = calib["src_pts"];
        std::vector<std::vector<double>> dst_angs = calib["dst_angs"];

        std::vector<double> s_ret_pts, s_ext_pts;
        for (int i = 1; i < src_pts.size(); i += 2) {
            s_ret_pts.push_back(src_pts[i][0]);
            s_ret_pts.push_back(src_pts[i][1]);
            s_ext_pts.push_back(src_pts[i + 1][0]);
            s_ext_pts.push_back(src_pts[i + 1][1]);
        }

        std::vector<double> arr(s_ret_pts.size());
        for (int i = 0; i < s_ret_pts.size(); i++) {
            arr[i] = s_ret_pts[i] - s_ext_pts[i];
        }

        // Define THETA_EXT and THETA_RET
        double THETA_EXT = 0.0;  // Replace with the actual values
        double THETA_RET = 0.0;  // Replace with the actual values

        double L = cv::norm(arr) / s_ret_pts.size() / (cos(THETA_EXT) - cos(THETA_RET));

        for (int i = 0; i < s_ret_pts.size(); i += 2) {
            arr[0] = s_ret_pts[i] - src_pts[0][0];
            arr[1] = s_ret_pts[i + 1] - src_pts[0][1];
        }

        double l1 = cv::norm(arr) / (s_ret_pts.size() / 2) - L * cos(THETA_RET);

        for (int i = 0; i < s_ext_pts.size(); i += 2) {
            arr[0] = s_ext_pts[i] - src_pts[0][0];
            arr[1] = s_ext_pts[i + 1] - src_pts[0][1];
        }

        double l2 = cv::norm(arr) / (s_ext_pts.size() / 2) - L * cos(THETA_EXT);

        double l = (l1 + l2) / 2;

        std::vector<std::vector<double>> dst_pts = {{0, 0}};

        for (const std::vector<double>& dst_ang : dst_angs) {
            double phi = dst_ang[0];
            double rho = L * cos(dst_ang[1]) + l;
            double x = rho * cos(phi);
            double y = rho * sin(phi);
            dst_pts.push_back({x, y});
        }

        cv::Mat src_pts_mat(src_pts.size(), 2, CV_64F);
        for (int i = 0; i < src_pts.size(); i++) {
            src_pts_mat.at<double>(i, 0) = src_pts[i][0];
            src_pts_mat.at<double>(i, 1) = src_pts[i][1];
        }

        cv::Mat dst_pts_mat(dst_pts.size(), 2, CV_64F);
        for (int i = 0; i < dst_pts.size(); i++) {
            dst_pts_mat.at<double>(i, 0) = dst_pts[i][0];
            dst_pts_mat.at<double>(i, 1) = dst_pts[i][1];
        }

        cv::Mat h = cv::findHomography(src_pts_mat, dst_pts_mat);
        // Assuming 'InvKin' and 'Arm3Link' classes are defined elsewhere
        std::vector<double> arm_vect = {L / 2, L / 2, l + L_FUDGE};
        InvKin = arm_vect;

        std::cout << "calibration loaded." << std::endl;
        std::cout << "estimated l = " << l << std::endl;
        std::cout << "estimated L = " << L << std::endl;

        cv::destroyAllWindows();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "calibration.json not in current directory, run calibration first" << std::endl;
    }
}
    
    std::tuple<float, float, float> TransformCoord(float x1, float y1, float r) {
    // void TransformCoord(double x1, double y1, double r){
        if (!homography_.empty()) {
            cv::Mat a = (cv::Mat_<float>(1, 2) << x1, y1);
            cv::Mat a_homogeneous;
            cv::convertPointsToHomogeneous(a, a_homogeneous);
            
            cv::Mat res;
            cv::perspectiveTransform(a_homogeneous, res, homography_);
            
            float x_res = res.at<float>(0, 0);
            float y_res = res.at<float>(0, 1);
            
            return std::make_tuple(x_res, y_res, DEFAULT_ROT);
        } 
        else {
            throw std::runtime_error("run or load calibration first!");
        }
    }
    
    void goJoint(double j0 = 0.0, double j1 = 0.0, double j2 = 0.0, double j3=0.0){
        std::vector<double> joint_goal = arm_group_.getCurrentJointValues();
        if (j0 >= 0)
            joint_goal[0] = j0;
        if (j1 >= 0)
            joint_goal[1] = j1;
        if (j2 >= 0)
            joint_goal[2] = j2;
        if (j3 >= 0)
            joint_goal[3] = j3;
        joint_goal[4] = 1.5708;

        arm_group_.setJointValueTarget(joint_goal);
        arm_group_.move();
        arm_group_.stop();
    }

    

    std::tuple<double, std::vector<double>>  getDown(double x,double y){
        std::pair<double, double> result;
        std::vector<double> xy;
        double s, phi;

        std::tie(s, phi) = cart2pol(x, y);
        std::vector<double> s_vec;
        s_vec[0]= s;

        std::vector<double> q = InvKin.inv_kin(s_vec, Z_MIN, Z_MAX_DOWN, -M_PI / 2);
        xy = InvKin.get_xy(q);

        if (std::abs(xy[0] - s) > CLOSE_ENOUGH) {
            std::cout << "NO SOLUTION FOUND" << std::endl;
            std::cout << "goal distance = " << s << std::endl;
            std::cout << "closest solution = " << xy[0] << std::endl;
            return std::make_tuple(s, std::vector<double>{s, phi, std::nan(""), std::nan(""), std::nan("")});
        } else {
            return std::make_tuple(s, std::vector<double>{phi, q[0], q[1] + M_PI / 2, q[2] + M_PI / 2});
        }
    }


    std::tuple<double, std::vector<double>> GetTarg(std::tuple<double, double> (double x, double y)){
        double x, y;
        std::pair<double, double> polar = cart2pol(x, y);
        std::vector<double> polar_vec = {polar.first, polar.second};
        std::vector<double> q = InvKin.inv_kin(polar_vec, Z_MIN, Z_MAX_SIDE, 0);
        std::vector<double> xy = InvKin.get_xy(q);

        if (std::abs(xy[0] - polar.first) > CLOSE_ENOUGH) {
            std::cout << "NO SOLUTION FOUND" << std::endl;
            std::cout << "goal distance = " << polar.first << std::endl;
            std::cout << "closest solution = " << xy[0] << std::endl;
            return std::make_tuple(polar_vec[0], std::vector<double>{polar_vec[1], std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()});
        }

        return std::make_tuple(polar_vec[0], std::vector<double>{polar_vec[1], q[0], q[1] + M_PI / 2, q[2] + M_PI / 2});

    }
    
    int goXY(double x, double y, double r, char bowl){
        double s;
        std::vector<double> joint_targets;
        std::make_tuple(s, joint_targets) = getDown(x,y);
        if (joint_targets[0] < 0 || joint_targets[0] > 3.14){
         std::cout << printf("++++++ Not in reachable area, aborting ++++++") << std::endl;
        return -1;
        }

        if (std::isnan(joint_targets[1])) {
        std::cout << "++++++ Not in reachable area, aborting ++++++" << std::endl;
        return -1;
        }
    
    goRaise();
    gripperOpen();
    
    goJoint(joint_targets[0], joint_targets[1], joint_targets[2], joint_targets[3]);

    gripperClosed();
    
    //TO_CONVERT PYTHON METHODS!
    // home_ch = getattr(self, bowl)
    // home_ch()
    
    return 0;
    }

    void goRaise(){
        std::vector<double> joint_goal = arm_group_.getCurrentJointValues();
        double j0 = joint_goal[0];
        double j1 = 1.5;
        double j2 = 0.13;
        double j3 = 2.29;
        goJoint(j0, j1, j2, j3);
    }

    void goPick(){
        std::vector<double> joint_goal = arm_group_.getCurrentJointValues();
        double j0 = joint_goal[0];
        double j1 = joint_goal[1];
        double j2 = 0.8;
        double j3 = joint_goal[3];
        goJoint(j0, j1, j2, j3);
    }

    void goPull(double phi){
        std::vector<double> joint_goal = arm_group_.getCurrentJointValues();
        goJoint(joint_goal[0], joint_goal[1], joint_goal[2], joint_goal[3]);
        goRaise();
        gripperClosed();
        if (phi){
            goJoint(phi, joint_goal[1], joint_goal[2], joint_goal[3]);
        
            goJoint(phi, 0.3,1.7,1.8);
            goJoint(phi, 0.3,1.5,0.3);
            goJoint(phi, 1.2,0.3,0.1);
            goJoint(phi, 1.5,0.3,0.1);
            goJoint(phi, 0.3,1.8,1.8);
        }
    }

    void goPush(double phi){
        std::vector<double> joint_goal = arm_group_.getCurrentJointValues();
        goJoint(joint_goal[0], joint_goal[1], joint_goal[2], joint_goal[3]);
        goRaise();
        gripperClosed();
        if (phi){
            goJoint(phi, 1.5, 0.13, 2.29);
        
        goJoint(phi, 2.7,0.01,0.01);
        goJoint(phi, 1.6,0.01,0.01);
        goJoint(phi, 0.3,1.8,0.1);
        goJoint(phi, 2.1,0.01,0.01);
        goJoint(phi, 2.7,0.01,0.01);
        }
    }


//methods for the gripper
    void gripperOpen(){
        std::vector<double> joint_goal = gripper.getCurrentJointValues();
        joint_goal[0] = 0.9;
        joint_goal[1] = 0.9;

        gripper.setJointValueTarget(joint_goal);
        gripper.move();
        gripper.stop();
    }

    void gripperClosed(){
        std::vector<double> joint_goal = gripper.getCurrentJointValues();
        joint_goal[0] = 1.2;
        joint_goal[1] = 1.2;

        gripper.setJointValueTarget(joint_goal);
        gripper.move();
        gripper.stop();
    }

    void gripperMiddle(){
        std::vector<double> joint_goal = gripper.getCurrentJointValues();
        joint_goal[0] = 1.05;
        joint_goal[1] = 1.05;

        gripper.setJointValueTarget(joint_goal);
        gripper.move();
        gripper.stop();
    }

    //METHODS TO PLACE THE FRAGMENT IN THE BOWL

    int goHome0(){
        double j0, j1, j2, j3;
        goPick();
        j0 = 2.355;
        std::vector<double> joint_goal = arm_group_.getCurrentJointValues();
        goJoint(j0, joint_goal[1], joint_goal[2], joint_goal[3]);
        goJoint(j0, 1.67, 0.10, 0.50);
        gripperOpen();
        return 0;

    }

    void goHome1(){
        double j0, j1, j2, j3;
        goPick();
        std::vector<double> joint_goal = arm_group_.getCurrentJointValues();
        j0 = 0.785;
        goJoint(j0, joint_goal[1], joint_goal[2], joint_goal[3]);
        goJoint(j0, 1.57, 3.00, 2.55);
        gripperOpen();
    }

    void goHome2(){
        double j0;
        goPick();
        std::vector<double> joint_goal = arm_group_.getCurrentJointValues();
        j0 = 2.355;
        goJoint(j0, joint_goal[1], joint_goal[2], joint_goal[3]);
        goJoint(j0, 1.47, 3.14, 2.50);
        gripperOpen();
    }

    void goUp(){
       
        goJoint(1.5708,1.5708,1.5708, 1.5708);
    }

    void goManual(){
        double j1;
        std::vector<double> joint_goal = arm_group_.getCurrentJointValues();
        
        goJoint(joint_goal[0], 2.5, joint_goal[2], joint_goal[3]);
    }

};


int main(int argc, char** argv){
    

    ros::init(argc, argv, "auto_targetter_step");
    ros::NodeHandle nh_;

    // ros::AsyncSpinner spin(1);
    // spin.start();
    
    BraccioObjectInterface br;

    
    ros::Subscriber sub = nh_.subscribe("/sherd_pub", 
            1000, &BraccioObjectInterface::callback_received_sherds, &br );
    


    ros::spin();

    // // br.loadCalibrate();
    // br.gripperClosed();
    // br.goRaise();
    // // br.goManual();
    // br.goHome0();
    // br.goRaise();
    // // br.goManual();
    // br.goHome1();
    // br.goRaise();
    // // br.goManual();
    // br.goHome2();


    // return 0;
}