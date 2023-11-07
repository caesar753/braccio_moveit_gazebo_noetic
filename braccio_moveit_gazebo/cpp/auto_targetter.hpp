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
#include <custom_msgs/target.h>
#include <custom_msgs/matrix.h>


#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <gazebo_msgs/LinkStates.h>

#include <nlohmann/json.hpp>
// #include <jsoncpp/json/json.h>
// #include <Eigen/Dense>

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

double calculateMeanNorm(const std::vector<std::vector<double>>& arr) {
    double sum = 0.0;

    for (const std::vector<double>& vec : arr) {
        double squaredSum = 0.0;

        for (double val : vec) {
            squaredSum += val * val;
        }

        sum += std::sqrt(squaredSum);
    }

    return sum / arr.size();
}

std::map<std::string, std::function<void()> > call;

class BraccioObjectInterface{
    
    private:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface arm_group_;
    moveit::planning_interface::MoveGroupInterface gripper;
    cv::Mat homography_;
    double L;
    double l;

    // tf2_ros::Buffer tf_buffer;
    // tf2_ros::TransformListener tf_listener(tf_buffer);
    gazebo_msgs::LinkStates linkstate_data;
    ros::Subscriber states_sub;
    std::string link_choose;
    std::vector<custom_msgs::matrix> targets_list;
    int i;
    ros::Subscriber target_matrix;


    public:
    BraccioObjectInterface() : nh_("~"), arm_group_("braccio_arm"), gripper("braccio_gripper"){
        moveit::planning_interface::MoveGroupInterface::Options arm_options("braccio_arm", "robot_description", nh_);
        moveit::planning_interface::MoveGroupInterface::Options gripper_options("braccio_gripper", "robot_description", nh_);
        // arm_group_ = moveit::planning_interface::MoveGroupInterface(arm_options);
        // gripper = moveit::planning_interface::MoveGroupInterface(gripper_options);

        targets_list.clear();// = std::vector<custom_msgs::matrix>(); // Replace YourMessageType with the appropriate message type
        i = 0;
        // target_matrix = nh_.subscribe("/targets", 1, &BraccioObjectInterface::callbackMatrix, this);
        // Sleep to allow ROS to get the robot state
        // ros::Duration(1.0).sleep();
        // Unregister the targets subscriber
        // target_matrix.shutdown();
    }

    // void callback_received_sherds(const custom_msgs::for_cpp& msg){

    //     ROS_INFO("Sherd_received; %s, %s", msg.sherds, msg.homes);
    // }

    // void callback_received_sherd_matrix(const custom_msgs::matrix& msg){

    //     ROS_INFO("Matrix_received; %s", msg.dati);
    // }



    void calibrate() {
        std::vector<std::vector<double>> src_pts;
        std::vector<std::vector<double>> dst_angs;

        // Replace with your actual implementation for getting link positions
        double mouseX, mouseY, r_;
        get_link_position(std::vector<std::string>{"kuka::base_link"});
        src_pts.push_back({mouseX, mouseY});

        gripperMiddle();
        int N = 8;
        double phi_min = M_PI / 6;
        double phi_max = M_PI - M_PI / 6;

        for (int i = 2; i < N; ++i) {
            goRaise();
            double rand_phi, theta_shoulder;
            if (i % 2 == 0) {
                rand_phi = phi_min + i * (phi_max - phi_min) / N;
                theta_shoulder = THETA_RET; // Define THETA_RET
            } else {
                theta_shoulder = THETA_EXT;
                srand((unsigned)time(0)); 
                int i;
                i = (rand()%8)+1; 
                rand_phi = phi_min + i * (phi_max - phi_min) / N;
            } // Define THETA_EXT}
            std::pair<double, double> theta_wrist, theta_elbow = get_other_angles(theta_shoulder);
            double theta_wrist_d, theta_elbow_d;
            std::tie (theta_wrist_d, theta_elbow_d) = (theta_wrist, theta_elbow);
            // , theta_elbow, theta_wrist);
            std::vector<double> rand_targ = {rand_phi, theta_shoulder, theta_elbow_d, theta_wrist_d};
            goJoint(rand_phi, theta_shoulder, theta_elbow_d, theta_wrist_d);

            // Replace with your actual implementation for getting link positions
            get_link_position(std::vector<std::string>{"kuka::left_gripper_link", "kuka::right_gripper_link"});
            src_pts.push_back({mouseX, mouseY});
            dst_angs.push_back(rand_targ);
            // }
        }

        std::ofstream file("calibration.json");
        if (file.is_open()) {
            json calibration_data = {{"src_pts", src_pts}, {"dst_angs", dst_angs}};
            file << calibration_data.dump(4);
            file.close();

            loadCalibrate();
            goUp();
        } else {
            std::cerr << "Failed to open calibration.json for writing" << std::endl;
        }
    }

    // void loadCalibrate() {
    // try {
    //     std::ifstream file("calibration.json");
    //     if (!file.is_open()) {
    //         std::cerr << "calibration.json not in current directory, run calibration first" << std::endl;
    //         // return;
    //         calibrate();
    //     }

    //     json calib = json::parse(file);
    //     // file >> calib;
    //     file.close();

    //     const auto s = calib.dump(); // serialize to std::string
    //     std::cout << "JSON string: " << s << '\n';
    //     std::cout << "JSON size  : " << s.size() << '\n';

    //     const json& src_pts_js = calib["src_pts"];
    //     const json& dst_angs_js = calib["dst_angs"];


    //     std::vector<std::vector<double>> src_pts;
    //     std::vector<std::vector<double>> dst_angs;

    //     // std::sprintf(src_pts[0]);

    //     for (const auto& point : src_pts_js) {
    //         src_pts.push_back({point[0].get<double>(), point[1].get<double>()});
    //     }

    //     std::vector<std::vector<double>> s_ret_pts, s_ext_pts;
    //     for (int i = 1; i < src_pts.size(); i += 2) {
    //         s_ret_pts.push_back(src_pts[i]);//[0]);
    //         // s_ret_pts.push_back(src_pts[i][1]);
    //         s_ext_pts.push_back(src_pts[i + 1]);//[0]);
    //         // s_ext_pts.push_back(src_pts[i + 1][1]);
    //     }

    //     std::vector<std::vector<double>> arr;//(s_ret_pts.size());
    //     for (int i = 0; i < s_ret_pts.size(); i++) {
    //         // arr[i] = s_ret_pts[i] - s_ext_pts[i];
    //         std::vector<double> arr_i(2);
    //         arr_i[0] = s_ret_pts[i][0] - s_ext_pts[i][0];
    //         arr_i[1] = s_ret_pts[i][1] - s_ext_pts[i][1];
    //         arr.push_back(arr_i);
    //     }

    //     double sum_of_squares = 0.0;
    //     for (const auto& v : arr) {
    //         sum_of_squares += v[0] * v[0] + v[1] * v[1];
    //     }

    //     // Define THETA_EXT and THETA_RET
    //     // double THETA_EXT = 0.0;  // Replace with the actual values
    //     // double THETA_RET = 0.0;  // Replace with the actual values

    //     // double L = cv::norm(arr) / s_ret_pts.size() / (cos(THETA_EXT) - cos(THETA_RET));
    //     double L = sqrt(sum_of_squares) / (cos(THETA_EXT) - cos(THETA_RET));

    //     // for (int i = 0; i < s_ret_pts.size(); i += 2) {
    //     //     arr[0] = s_ret_pts[i] - src_pts[0][0];
    //     //     arr[1] = s_ret_pts[i + 1] - src_pts[0][1];
    //     // }
    //     std::vector<std::vector<double>> arr1;
    //     for (size_t i = 0; i < s_ret_pts.size(); ++i) {
    //         std::vector<double> arr1_i(2);
    //         arr1_i[0] = s_ret_pts[i][0] - src_pts[0][0];
    //         arr1_i[1] = s_ret_pts[i][1] - src_pts[0][1];
    //         arr1.push_back(arr1_i);
    //     }

    //     // double l1 = cv::norm(arr) / (s_ret_pts.size() / 2) - L * cos(THETA_RET);

    //     // for (int i = 0; i < s_ext_pts.size(); i += 2) {
    //     //     arr[0] = s_ext_pts[i] - src_pts[0][0];
    //     //     arr[1] = s_ext_pts[i + 1] - src_pts[0][1];
    //     // }

    //     // double l2 = cv::norm(arr) / (s_ext_pts.size() / 2) - L * cos(THETA_EXT);

    //     // double l = (l1 + l2) / 2;

    //     double l1 = 0.0;
    //     for (const auto& v : arr1) {
    //         l1 += sqrt(v[0] * v[0] + v[1] * v[1]);
    //     }
    //     l1 /= s_ret_pts.size();

    //     std::vector<std::vector<double>> arr2;
    //     for (size_t i = 0; i < s_ext_pts.size(); ++i) {
    //         std::vector<double> arr2_i(2);
    //         arr2_i[0] = s_ext_pts[i][0] - src_pts[0][0];
    //         arr2_i[1] = s_ext_pts[i][1] - src_pts[0][1];
    //         arr2.push_back(arr2_i);
    //     }

    //     double l2 = 0.0;
    //     for (const auto& v : arr2) {
    //         l2 += sqrt(v[0] * v[0] + v[1] * v[1]);
    //     }
    //     l2 /= s_ext_pts.size();

    //     double l = (l1 + l2) / 2;

    //     std::vector<std::vector<double>> dst_pts = {{0, 0}};

    //     for (const std::vector<double>& dst_ang : dst_angs) {
    //         double phi = dst_ang[0];
    //         double rho = L * cos(dst_ang[1]) + l;
    //         double x = rho * cos(phi);
    //         double y = rho * sin(phi);
    //         dst_pts.push_back({x, y});
    //     }

    //     cv::Mat src_pts_mat(src_pts.size(), 2, CV_64F);
    //     for (int i = 0; i < src_pts.size(); i++) {
    //         src_pts_mat.at<double>(i, 0) = src_pts[i][0];
    //         src_pts_mat.at<double>(i, 1) = src_pts[i][1];
    //     }

    //     cv::Mat dst_pts_mat(dst_pts.size(), 2, CV_64F);
    //     for (int i = 0; i < dst_pts.size(); i++) {
    //         dst_pts_mat.at<double>(i, 0) = dst_pts[i][0];
    //         dst_pts_mat.at<double>(i, 1) = dst_pts[i][1];
    //     }

    //     cv::Mat h = cv::findHomography(src_pts_mat, dst_pts_mat, cv::RANSAC);
    //     this->homography_ = h;

    // void loadCalibrate() {
    //     try {
    //         Json::Value calib;
    //         std::ifstream file("calibration.json");

    //         if (!file.is_open()) {
    //             std::cerr << "Failed to open calibration.json" << std::endl;
    //             return;
    //         }

    //         file >> calib;
    //         file.close();

    //         const Json::Value src_pts = calib["src_pts"];
    //         const Json::Value dst_angs = calib["dst_angs"];

    //         Eigen::MatrixXd src_pts_matrix(src_pts.size(), 2);
    //         Eigen::MatrixXd dst_pts_matrix(dst_angs.size() + 1, 2);

    //         for (int i = 0; i < src_pts.size(); ++i) {
    //             src_pts_matrix(i, 0) = src_pts[i][0].asDouble();
    //             src_pts_matrix(i, 1) = src_pts[i][1].asDouble();
    //         }

    //         Eigen::MatrixXd s_ret_pts = src_pts_matrix.block(1, 0, src_pts_matrix.rows() - 1, src_pts_matrix.cols());
    //         Eigen::MatrixXd s_ext_pts = src_pts_matrix.block(2, 0, src_pts_matrix.rows() - 2, src_pts_matrix.cols());
            
    //         Eigen::MatrixXd arr = s_ret_pts - s_ext_pts;
    //         double L = sqrt(arr.array().square().rowwise().sum().mean()) / (cos(THETA_EXT) - cos(THETA_RET));

    //         Eigen::MatrixXd arr1 = s_ret_pts - src_pts_matrix.row(0).transpose();
    //         double l1 = sqrt(arr1.array().square().rowwise().sum().mean()) - L * cos(THETA_RET);
    //         Eigen::MatrixXd arr2 = s_ext_pts - src_pts_matrix.row(0).transpose();
    //         double l2 = sqrt(arr2.array().square().rowwise().sum().mean()) - L * cos(THETA_EXT);
    //         double l = (l1 + l2) / 2;

    //         dst_pts_matrix(0, 0) = 0;
    //         dst_pts_matrix(0, 1) = 0;

    //         for (int i = 0; i < dst_angs.size(); ++i) {
    //             double phi = dst_angs[i][0].asDouble();
    //             double rho = L * cos(dst_angs[i][1].asDouble()) + l;
    //             double x = rho * cos(phi);
    //             double y = rho * sin(phi);
    //             dst_pts_matrix(i + 1, 0) = x;
    //             dst_pts_matrix(i + 1, 1) = y;
    //         }

    //         cv::Mat H = cv::findHomography(src_pts_matrix, dst_pts_matrix, cv::RANSAC);

    //         // Store the homography matrix for further use
    //         this->homography_ = H;
        // Assuming 'InvKin' and 'Arm3Link' classes are defined elsewhere
//         std::vector<double> arm_vect = {L / 2, L / 2, l + L_FUDGE};
//         InvKin = arm_vect;

//         std::cout << "calibration loaded." << std::endl;
//         std::cout << "estimated l = " << l << std::endl;
//         std::cout << "estimated L = " << L << std::endl;

//         cv::destroyAllWindows();
//     } catch (const std::exception& e) {
//         std::cerr << "Error: " << e.what() << std::endl;
//         std::cerr << "calibration.json not in current directory, run calibration first" << std::endl;
//     }
// }
    void loadCalibrate() {
        try {
            std::ifstream file("calibration.json");
            if (!file.is_open()) {
                std::cerr << "calibration.json not in current directory, run calibration first" << std::endl;
                return;
            }

            json calib = json::parse(file);
            file.close();

            const auto s = calib.dump(); // serialize to std::string
            std::cout << "JSON string: " << s << '\n';
            std::cout << "JSON size  : " << s.size() << '\n';

            const json& src_pts_js = calib["src_pts"];
            const json& dst_angs_js = calib["dst_angs"];


            std::vector<std::vector<double>> src_pts;
            std::vector<std::vector<double>> dst_angs;

            for (const auto& point : src_pts_js) {
                src_pts.push_back({point[0].get<double>(), point[1].get<double>()});
            }

            for (const auto& point2 : dst_angs_js) {
                dst_angs.push_back({point2[0].get<double>(), point2[1].get<double>(), point2[2].get<double>(),point2[3].get<double>()});
            }

            std::cout << "dst_angs is " << std::endl;
            for(int a = 0; a < dst_angs.size(); a++){
                for(int b = 0; b < dst_angs[a].size(); b++){
                    std::cout << dst_angs[a][b] << ", ";
                    }
                std::cout << std::endl;
                }

            std::vector<std::vector<double>> s_ret_pts, s_ext_pts;
            for (int i = 1; i < src_pts.size(); i += 2) {
                s_ret_pts.push_back(src_pts[i]);
                s_ext_pts.push_back(src_pts[i + 1]);
            }

            std::vector<std::vector<double>> arr;//(s_ret_pts.size());
            for (int i = 0; i < s_ret_pts.size(); i++) {
                // arr[i] = s_ret_pts[i] - s_ext_pts[i];
                std::vector<double> arr_i(2);
                arr_i[0] = s_ret_pts[i][0] - s_ext_pts[i][0];
                arr_i[1] = s_ret_pts[i][1] - s_ext_pts[i][1];
                arr.push_back(arr_i);
            }

            // double sum_of_squares = 0.0;
            // for (const auto& v : arr) {
            //     sum_of_squares += v[0] * v[0] + v[1] * v[1];
            // }
            L = calculateMeanNorm(arr) / (cos(THETA_EXT) - cos(THETA_RET));
            // double L = sqrt(sum_of_squares) / (cos(THETA_EXT) - cos(THETA_RET));
            std::cout << "L is" << L << std::endl;

            std::vector<std::vector<double>> arr1;
            for (size_t i = 0; i < s_ret_pts.size(); ++i) {
                std::vector<double> arr1_i(2);
                arr1_i[0] = s_ret_pts[i][0] - src_pts[0][0];
                arr1_i[1] = s_ret_pts[i][1] - src_pts[0][1];
                arr1.push_back(arr1_i);
            }

            // double l1 = 0.0;
            // for (const auto& v : arr1) {
            //     l1 += sqrt(v[0] * v[0] + v[1] * v[1]);
            // }
            // std::cout << "l1 before is" << l1 << std::endl;
            // l1 /= s_ret_pts.size();
            // std::cout << "l1 after is" << l1 << std::endl;
            double l1 = calculateMeanNorm(arr1);
            std::cout << "l1 before is" << l1 << std::endl;
            // l1 /= s_ret_pts.size();
            // std::cout << "l1 after is" << l1 << std::endl;
            l1 = l1 - L*cos(THETA_RET);
            std::cout << "l1 after is" << l1 << std::endl;

            std::vector<std::vector<double>> arr2;
            for (size_t i = 0; i < s_ext_pts.size(); ++i) {
                std::vector<double> arr2_i(2);
                arr2_i[0] = s_ext_pts[i][0] - src_pts[0][0];
                arr2_i[1] = s_ext_pts[i][1] - src_pts[0][1];
                arr2.push_back(arr2_i);
            }

            // double l2 = 0.0;
            // for (const auto& v : arr2) {
            //     l2 += sqrt(v[0] * v[0] + v[1] * v[1]);
            // }
            // std::cout << "l2 before is" << l2 << std::endl;
            // l2 /= s_ext_pts.size();
            // std::cout << "l2 after is" << l2 << std::endl;
            double l2 = calculateMeanNorm(arr2);
            std::cout << "l2 before is" << l2 << std::endl;
            // l2 /= s_ext_pts.size();
            // std::cout << "l2 after is" << l2 << std::endl;
            l2 = l2 - L*cos(THETA_EXT);
            std::cout << "l2 after is" << l2 << std::endl;

            l = (l1 + l2)/ 2;
            std::cout << "l is" << l << std::endl;

            std::vector<std::vector<double>> dst_pts = {{0, 0}};

            for (const std::vector<double>& dst_ang : dst_angs) {
                // double phi = dst_ang[0];
                // std::cout << "phi is "<< phi << std::endl;
                // double rho = L * cos(dst_ang[1]) + l;
                // std::cout << "rho is "<< rho << std::endl;
                // double x = rho * cos(phi);
                // std::cout << "x is "<< x << std::endl;
                // double y = rho * sin(phi);
                // std::cout << "y is "<< y << std::endl;
                // dst_pts.push_back({x, y});
                double phi = dst_ang[0];
                std::cout << "phi is" << phi << std::endl;
                double rho = L * cos(dst_ang[1]) + l;
                std::cout << "rho is" << rho << std::endl;
                std::pair<double,double> coord = pol2cart(rho, phi);
                dst_pts.push_back({coord.first, coord.second});
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

            int rows_src = sizeof(src_pts)/sizeof(src_pts[0]);
            int cols_src = sizeof(src_pts[0])/sizeof(src_pts[0][0]);
            std::cout<< "Shape of src_pts is "<< rows_src <<" "<<cols_src<<std::endl;

            int rows_dst = sizeof(dst_pts)/sizeof(dst_pts[0]);
            int cols_dst = sizeof(dst_pts[0])/sizeof(dst_pts[0][0]);
            std::cout<< "Shape of dst_pts is "<< rows_dst <<" "<< cols_dst <<std::endl;

            std::cout << "src_pts array is " << std::endl;
            for(int i = 0; i < src_pts.size(); i++){
                for(int j = 0; j < src_pts[i].size(); j++){
                    std::cout << src_pts[i][j] << ", ";
                    }
                std::cout << std::endl;
                }

            std::cout << "dst_pts array is " << std::endl;
            for(int h = 0; h < dst_pts.size(); h++){
                for(int k = 0; k < dst_pts[h].size(); k++){
                    std::cout <<  dst_pts[h][k] << ", ";
                    }
                std::cout << std::endl;
                }

            cv::Mat h = cv::findHomography(src_pts_mat, dst_pts_mat, cv::RANSAC);
            homography_ = h;

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
    
       void linkstate_callback(const gazebo_msgs::LinkStates::ConstPtr& data) {
        linkstate_data = *data;
    }



    std::tuple<double, double, double> get_box_position() {
        // x, y, r = self.get_link_position(['unit_box_1::link'])
        std::vector<std::string> link_names = {link_choose};
        std::tuple<double, double, double> result = get_link_position(link_names);
        std::tuple<double, double, double> transformed = TransformCoord(std::get<0>(result), std::get<1>(result), std::get<2>(result));
        return transformed;
    }

    void get_link_choose(const std::string& lk) {
        link_choose = lk;
        std::cout << link_choose << std::endl;
        std::cout << link_choose << std::endl;
    }

    std::tuple<double, double, double> get_link_position(const std::vector<std::string>& link_names) {
        double x = 0;
        double y = 0;
        int n = 0;
        arm_group_.getCurrentJointValues();
        gripper.getCurrentJointValues();
        for (const std::string& l : link_names) {
            auto it = std::find(linkstate_data.name.begin(), linkstate_data.name.end(), l);
            if (it != linkstate_data.name.end()) {
                int ind = std::distance(linkstate_data.name.begin(), it);
                geometry_msgs::Pose pose = linkstate_data.pose[ind];
                x += pose.position.x;
                y += pose.position.y;
                n += 1;
            }
        }
        double rotation = DEFAULT_ROT;
        return std::make_tuple(x / n, y / n, rotation);
    }


    // int goTarget(const std::string& how, const std::string& bowl) {
        int goTarget(const std::string& bowl){
        std::tuple<double, double, double> result = get_box_position();
        double x = std::get<0>(result);
        double y = std::get<1>(result);
        double r = std::get<2>(result);

        std::cout << x << " " << y << " " << r << std::endl;

        std::string bowl2 = bowl;
        return goXY(x, y, r,  bowl2);
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
    
    // int go_to_xy(double x, double y, double r, const std::string& how, const std::string& bowl) {
    //     std::vector<double> joint_goal = arm_group_.getCurrentJointValues();

        
    //     if (how == "top") {
    //         std::tuple<double, std::vector<double>> result = get_down_targets(x, y);
    //         double s = std::get<0>(result);
    //         joint_goal = std::get<1>(result);

    //         if (joint_targets[0] < 0 || joint_targets[0] > 3.14) {
    //             std::cout << "++++++ Not in reachable area, aborting ++++++" << std::endl;
    //             return -1;
    //         }

    //         if (std::isnan(joint_targets[1])) {
    //             std::cout << "++++++ Not in reachable area, aborting ++++++" << std::endl;
    //             return -1;
    //         }
    //     }
        
    //     // Implement the rest of the go_to_xy method logic here
    //     // self.go_to_raise()

    //     // self.gripper_open()
    //     // self.go_to_j(j0=float(joint_targets[0]))

    //     // self.go_to_j(j1=float(joint_targets[1]),
    //     //             j2=float(joint_targets[2]),
    //     //             j3=float(joint_targets[3]))

    //     // self.gripper_close()
    //     // # if how=='top' and joint_targets[2]<3:
    //     // #   self.go_to_j(j2=float(joint_targets[2])+0.1)
    //     // # self.go_to_home()
    //     // home_ch = getattr(self, bowl)
    //     // home_ch()

    //     goRaise();

    //     gripperOpen();
        
    //     goJoint(double j0 = joint_goal[0]);

    //     goJoint(joint_targets[0], joint_targets[1], joint_targets[2], joint_targets[3]);

    //     gripperClosed();
        

    //     return 0;
    // }
    
    int goXY(double x, double y, double r, std::string bowl2){
    // int goXY(double x, double y, double r){
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

    // call[bowl2] = bowl2;
    // call[bowl2]();
    
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

    // void callbackMatrix(const custom_msgs::matrix::ConstPtr& msg) {
    //     for (size_t i = 0; i < msg->targets.size(); ++i) {
    //         targets_list.push_back(msg->targets[i]);
    //         this->i = i;
    //     }
    // }

    // std::pair<int, std::vector<custom_msgs::matrix>> returnTargets() {
    //     return std::make_pair(i, targets_list);
    // }

};