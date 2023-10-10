#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <jsoncpp/json/json.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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

//function which transforms cartesian coord into polar coord
std::pair<double, double> cart2pol(double x, double y) {
  // helper, convert cartesian to polar coordinates
  double rho = std::sqrt(x * x + y * y);
  double phi = std::atan2(y, x);
  return std::make_pair(rho, phi);
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
        arm_group_ = moveit::planning_interface::MoveGroupInterface(arm_options);
        gripper = moveit::planning_interface::MoveGroupInterface(gripper_options);
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
    
    void goJoint(double j0 = 0.0, double j1 = 0.3, double j2 = 1.7, double j3=1.8){
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

    void goRaise(){
        double j1 = 1.5;
        double j2 = 0.13;
        double j3 = 2.29;
        goJoint(j1, j2, j3);
    }

    void goPick(){
        double j1 = 1.5;
        // double j2 = 0.8;
        // double j3 = 2.29;
        // goJoint(j1, j2, j3);
        goJoint(j1);
    }

    void goPull(double phi){
        goRaise();
        gripperClosed();
        if (phi){
            goJoint(phi, 1.5, 0.13, 2.29);
        };
        goJoint(0.3,1.7,1.8);
        goJoint(0.3,1.5,0.3);
        goJoint(1.2,0.3,0.1);
        goJoint(1.5,0.3,0.1);
        goJoint(0.3,1.8,1.8);
    }

    void goPush(double phi){
        goRaise();
        gripperClosed();
        if (phi){
            goJoint(phi, 1.5, 0.13, 2.29);
        };
        goJoint(2.7,0.01,0.01);
        goJoint(1.6,0.01,0.01);
        goJoint(0.3,1.8,0.1);
        goJoint(2.1,0.01,0.01);
        goJoint(2.7,0.01,0.01);
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

};

int main(int argc, char** argv){
    

    ros::init(argc, argv, "auto_targetter_step");
    ros::NodeHandle nh_;

    ros::AsyncSpinner spin(1);
    spin.start();

    std::printf("sono qua");
    
    BraccioObjectInterface br;


    std::printf("ma non qua");
    

    br.gripperMiddle();
    return 0;
}