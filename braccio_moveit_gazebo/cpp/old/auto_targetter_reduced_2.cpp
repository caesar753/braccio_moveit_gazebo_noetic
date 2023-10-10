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



std::pair<double, double> cart2pol(double x, double y) {
  // helper, convert cartesian to polar coordinates
  double rho = std::sqrt(x * x + y * y);
  double phi = std::atan2(y, x);
  return std::make_pair(rho, phi);
}

std::pair<double, double> pol2cart(double rho, double phi) {
  // helper, convert polar to cartesian coordinates
  double x = rho * std::cos(phi);
  double y = rho * std::sin(phi);
  return std::make_pair(x, y);
}

std::pair<double, double> get_other_angles(double theta_shoulder) {
  // helper, converting some angles
  double theta_wrist = theta_shoulder + M_PI / 2;
  double theta_elbow = M_PI / 2 - 2 * theta_shoulder;
  return std::make_pair(theta_wrist, theta_elbow);
}

// namespace braccio_arm_control
// {
    class BraccioObjectTargetInterface
    {
    private:
        ros::NodeHandle nh_;
        moveit::planning_interface::MoveGroupInterface arm_group_;
        moveit::planning_interface::MoveGroupInterface gripper_group_;
        cv::Mat homography_;
        // InvKin::Arm3Link kinematics_;
        Arm3Link kinematics_;
    
    public:
        BraccioObjectTargetInterface() : nh_("~"), arm_group_("braccio_arm"), gripper_group_("braccio_gripper")
        {
            moveit::planning_interface::MoveGroupInterface::Options arm_options("braccio_arm", "robot_description", nh_);
            moveit::planning_interface::MoveGroupInterface::Options gripper_options("braccio_gripper", "robot_description", nh_);
            arm_group_ = moveit::planning_interface::MoveGroupInterface(arm_options);
            gripper_group_ = moveit::planning_interface::MoveGroupInterface(gripper_options);
        }

        ~BraccioObjectTargetInterface()
        {
        }

        // void loadCalibration()
        // {
        //     std::string package_path = ros::package::getPath("braccio_arm_control");
        //     std::string calibration_file = package_path + "/calibration.json";

        //     std::ifstream file(calibration_file);
        //     if (!file.is_open())
        //     {
        //         ROS_ERROR_STREAM("Failed to open calibration file: " << calibration_file);
        //         return;
        //     }

        //     std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        //     file.close();

        //     Json::Value calib;
        //     Json::Reader reader;
        //     if (!reader.parse(content, calib))
        //     {
        //         ROS_ERROR_STREAM("Failed to parse calibration file: " << calibration_file);
        //         return;
        //     }

        //     Json::Value src_pts_json = calib["src_pts"];
        //     Json::Value dst_angs_json = calib["dst_angs"];

        //     std::vector<cv::Point2f> src_pts, dst_pts;
        //     for (const auto &pt : src_pts_json)
        //     {
        //         src_pts.push_back(cv::Point2f(pt[0].asFloat(), pt[1].asFloat()));
        //     }

        //     dst_pts.push_back(cv::Point2f(0, 0));
        //     for (const auto &ang : dst_angs_json)
        //     {
        //         float phi = ang[0].asFloat();
        //         float rho = L_ * std::cos(ang[1].asFloat()) + l_;
        //         float x = rho * std::cos(phi);
        //         float y = rho * std::sin(phi);
        //         dst_pts.push_back(cv::Point2f(x, y));
        //     }

        //     homography_ = cv::findHomography(src_pts, dst_pts);
        //     // kinematics_ = InvKin::Arm3Link(L_);
        //     kinematics_ = Arm3Link(L_);

        //     ROS_INFO_STREAM("Calibration loaded.");
        //     ROS_INFO_STREAM("Estimated l = " << l_);
        //     ROS_INFO_STREAM("Estimated L = " << L_);
        // }

    void load_calibrate() {
        try {
            std::string package_path = ros::package::getPath("braccio_arm_control");
            std::string calibration_file = package_path + "/calibration.json";

            std::ifstream file(calibration_file);
            if (!file.is_open())
            {
                ROS_ERROR_STREAM("Failed to open calibration file: " << calibration_file);
                return;
            }

            std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
            file.close();

            Json::Value calib;
            Json::Reader reader;
            if (!reader.parse(content, calib))
            {
                ROS_ERROR_STREAM("Failed to parse calibration file: " << calibration_file);
                return;
            }

            Json::Value src_pts = calib["src_pts"];
            Json::Value dst_angs = calib["dst_angs"];
            
            
           //NEW GENERATED 
            // std::ifstream file("calibration.json");
            // if (file.is_open()) {
            //     Json calib;
            //     file >> calib;
            //     file.close();

            //     std::vector<std::vector<double>> src_pts = calib["src_pts"];
            //     std::vector<std::vector<double>> dst_angs = calib["dst_angs"];

                std::vector<std::vector<double>> s_ret_pts;
                std::vector<std::vector<double>> s_ext_pts;
                for (size_t i = 1; i < src_pts.size(); i += 2) {
                    s_ret_pts.push_back(src_pts[i]);
                    s_ext_pts.push_back(src_pts[i + 1]);
                }

                std::vector<std::vector<double>> arr(s_ret_pts.size(), std::vector<double>(2));
                for (size_t i = 0; i < s_ret_pts.size(); i++) {
                    arr[i][0] = s_ret_pts[i][0] - s_ext_pts[i][0];
                    arr[i][1] = s_ret_pts[i][1] - s_ext_pts[i][1];
                }

                double sum = 0.0;
                for (const auto& row : arr) {
                    sum += std::sqrt(row[0] * row[0] + row[1] * row[1]);
                }
                double L = sum / arr.size() / (std::cos(THETA_EXT) - std::cos(THETA_RET));

                arr.clear();
                arr.resize(s_ret_pts.size(), std::vector<double>(2));
                for (size_t i = 0; i < s_ret_pts.size(); i++) {
                    arr[i][0] = s_ret_pts[i][0] - src_pts[0][0];
                    arr[i][1] = s_ret_pts[i][1] - src_pts[0][1];
                }
                double l1 = 0.0;
                for (const auto& row : arr) {
                    l1 += std::sqrt(row[0] * row[0] + row[1] * row[1]);
                }
                l1 /= arr.size();
                l1 -= L * std::cos(THETA_RET);

                arr.clear();
                arr.resize(s_ext_pts.size(), std::vector<double>(2));
                for (size_t i = 0; i < s_ext_pts.size(); i++) {
                    arr[i][0] = s_ext_pts[i][0] - src_pts[0][0];
                    arr[i][1] = s_ext_pts[i][1] - src_pts[0][1];
                }
                double l2 = 0.0;
                for (const auto& row : arr) {
                    l2 += std::sqrt(row[0] * row[0] + row[1] * row[1]);
                }
                l2 /= arr.size();
                l2 -= L * std::cos(THETA_EXT);

                double l = (l1 + l2) / 2.0;

                std::vector<std::vector<double>> dst_pts;
                dst_pts.push_back({0, 0});
                for (const auto& dst_ang : dst_angs) {
                    double phi = dst_ang[0];
                    double rho = L * std::cos(dst_ang[1]) + l;
                    double x, y;
                    pol2cart(rho, phi, x, y);
                    dst_pts.push_back({x, y});
                }

                cv::Mat src_pts_mat(src_pts.size(), 2, CV_64F);
                cv::Mat dst_pts_mat(dst_pts.size(), 2, CV_64F);
                for (size_t i = 0; i < src_pts.size(); i++) {
                    src_pts_mat.at<double>(i, 0) = src_pts[i][0];
                    src_pts_mat.at<double>(i, 1) = src_pts[i][1];
                }
                for (size_t i = 0; i < dst_pts.size(); i++) {
                    dst_pts_mat.at<double>(i, 0) = dst_pts[i][0];
                    dst_pts_mat.at<double>(i, 1) = dst_pts[i][1];
                }

                cv::Mat h = cv::findHomography(src_pts_mat, dst_pts_mat);

                // Assuming InvKin.Arm3Link is a class with a constructor that takes an array as an argument
                // InvKin.Arm3Link kinematics({L / 2, L / 2, l + L_FUDGE});
                kinematics_ = Arm3Link(L_);

                std::cout << "calibration loaded." << std::endl;
                std::cout << "estimated l = " << l << std::endl;
                std::cout << "estimated L = " << L << std::endl;

                cv::destroyAllWindows();
            } else {
                std::cout << "calibration.json not in current directory, run calibration first" << std::endl;
            }
        } catch (...) {
            std::cout << "An error occurred while loading calibration." << std::endl;
        }
    }


        void goHome()
        {
            goPick();
            goJoint(2.355, 1.67, 0.10, 0.5);
            gripperOpen();
            gripperOpen();
        }

        void goPick()
        {
            // TODO: Implement goPick
        }

        void goJoint(double j0, double j1, double j2, double j3)
        {
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

        void gripperOpen()
        {
            std::vector<double> joint_goal = gripper_group_.getCurrentJointValues();
            joint_goal[0] = 1.2;
            joint_goal[1] = 1.2;

            gripper_group_.setJointValueTarget(joint_goal);
            gripper_group_.move();
            gripper_group_.stop();
        }

        void linkch()
        {
            std::ifstream group_file("choosen.txt");
            std::ifstream position_file("posizioni_mm.txt");

            std::vector<std::string> groups;
            std::vector<std::vector<std::string>> positions;

            if (!group_file.is_open())
            {
                ROS_ERROR_STREAM("Failed to open choosen.txt");
                return;
            }

            if (!position_file.is_open())
            {
                ROS_ERROR_STREAM("Failed to open posizioni_mm.txt");
                return;
            }

            std::string line;
            while (std::getline(group_file, line))
            {
                std::istringstream iss(line);
                std::string group;
                while (iss >> group)
                {
                    groups.push_back(group);
                }
            }
            group_file.close();

            while (std::getline(position_file, line))
            {
                std::istringstream iss(line);
                std::vector<std::string> position;
                std::string value;
                while (iss >> value)
                {
                    position.push_back(value);
                }
                positions.push_back(position);
            }
            position_file.close();

            std::vector<std::string> link_choose;
            for (size_t i = 0; i < positions.size(); ++i)
            {
                if (std::find(groups.begin(), groups.end(), positions[i][0]) != groups.end())
                {
                    link_choose.push_back(positions[i][4]);
                }
            }

            // TODO: Use link_choose as needed
        }

    };
// } // namespace BraccioObjectTargetInterface
