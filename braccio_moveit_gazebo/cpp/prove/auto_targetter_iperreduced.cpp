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

#include "../InvKin.hpp"

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
//   return {rho, phi};
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
        states_sub = nh_.subscribe("/gazebo/link_states", 1, &BraccioObjectInterface::linkstate_callback, this);
        // target_matrix = nh_.subscribe("/targets", 1, &BraccioObjectInterface::callbackMatrix, this);
        // Sleep to allow ROS to get the robot state
        // ros::Duration(1.0).sleep();
        // Unregister the targets subscriber
        // target_matrix.shutdown();
    }

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

            L = calculateMeanNorm(arr) / (cos(THETA_EXT) - cos(THETA_RET));
            std::cout << "L is" << L << std::endl;

            std::vector<std::vector<double>> arr1;
            for (size_t i = 0; i < s_ret_pts.size(); ++i) {
                std::vector<double> arr1_i(2);
                arr1_i[0] = s_ret_pts[i][0] - src_pts[0][0];
                arr1_i[1] = s_ret_pts[i][1] - src_pts[0][1];
                arr1.push_back(arr1_i);
            }

            
            double l1 = calculateMeanNorm(arr1);
            std::cout << "l1 before is" << l1 << std::endl;
            l1 = l1 - L*cos(THETA_RET);
            std::cout << "l1 after is" << l1 << std::endl;

            std::vector<std::vector<double>> arr2;
            for (size_t i = 0; i < s_ext_pts.size(); ++i) {
                std::vector<double> arr2_i(2);
                arr2_i[0] = s_ext_pts[i][0] - src_pts[0][0];
                arr2_i[1] = s_ext_pts[i][1] - src_pts[0][1];
                arr2.push_back(arr2_i);
            }

            double l2 = calculateMeanNorm(arr2);
            std::cout << "l2 before is" << l2 << std::endl;
            l2 = l2 - L*cos(THETA_EXT);
            std::cout << "l2 after is" << l2 << std::endl;

            l = (l1 + l2)/ 2;
            std::cout << "l is" << l << std::endl;

            std::vector<std::vector<double>> dst_pts = {{0, 0}};

            for (const std::vector<double>& dst_ang : dst_angs) {
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
         try {
            linkstate_data = *data;
            } catch (const std::exception& e) {
            ROS_ERROR("Error: %s", e.what());
        }
    }

    std::tuple<double, double, double> get_box_position() {
        // x, y, r = self.get_link_position(['unit_box_1::link'])
        std::vector<std::string> link_names = {link_choose};
        std::cout << "link_choose in get_box_position method is" << link_names[0] << std::endl;
        std::tuple<double, double, double> result = get_link_position(link_names);
        std::cout << "Result is " << std::get<0>(result) << ","  << std::get<1>(result) << "," << std::get<2>(result) << std::endl;
        std::tuple<double, double, double> transformed = TransformCoord(std::get<0>(result), std::get<1>(result), std::get<2>(result));
        std::cout << "Transformed is " << std::get<0>(result) << ","  << std::get<1>(result) << "," << std::get<2>(result) << std::endl;
        return transformed;
        // return result;
    }

    void get_link_choose(const std::string& lk) {
        link_choose = lk;
        std::cout << link_choose << std::endl;
        std::cout << link_choose << std::endl;
    }

    std::tuple<double, double, double> get_link_position(const std::vector<std::string>& link_names) {
    // std::tuple<double, double, double> get_link_position(std::string& link_names)
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
    if (!homography_.empty()) {
        std::vector<cv::Point2f> points_in = {cv::Point2f(x1, y1)};
        std::vector<cv::Point2f> points_out;

        cv::perspectiveTransform(points_in, points_out, homography_);

        float x_res = points_out[0].x;
        float y_res = points_out[0].y;

        return std::make_tuple(x_res, y_res, DEFAULT_ROT);
    } else {
        throw std::runtime_error("Run or load calibration first!");
      }
  }

  // std::tuple<double, std::vector<double>>  getDown(double x,double y){
  //       std::pair<double, double> result;
  //       std::vector<double> xy;
  //       double s, phi;

  //       std::tie(s, phi) = cart2pol(x, y);
  //       std::vector<double> s_vec;
  //       s_vec[0]= s;

  //       std::vector<double> q = InvKin.inv_kin(s_vec, Z_MIN, Z_MAX_DOWN, -M_PI / 2);
  //       xy = InvKin.get_xy(q);

  //       if (std::abs(xy[0] - s) > CLOSE_ENOUGH) {
  //           std::cout << "NO SOLUTION FOUND" << std::endl;
  //           std::cout << "goal distance = " << s << std::endl;
  //           std::cout << "closest solution = " << xy[0] << std::endl;
  //           return std::make_tuple(s, std::vector<double>{s, phi, std::nan(""), std::nan(""), std::nan("")});
  //       } else {
  //           return std::make_tuple(s, std::vector<double>{phi, q[0], q[1] + M_PI / 2, q[2] + M_PI / 2});
  //       }
  //   }

    int goXY(double x, double y, double r, std::string bowl2){
    // int goXY(double x, double y, double r){
        double s;
        std::vector<double> joint_targets;
        // std::make_tuple(s, joint_targets) = getDown(x,y);
        // if (joint_targets[0] < 0 || joint_targets[0] > 3.14){
        //  std::cout << printf("++++++ Not in reachable area, aborting ++++++") << std::endl;
        // return -1;
        // }

        // if (std::isnan(joint_targets[1])) {
        // std::cout << "++++++ Not in reachable area, aborting ++++++" << std::endl;
        // return -1;
        // }
    
    // goRaise();
    // gripperOpen();
    
    // goJoint(joint_targets[0], joint_targets[1], joint_targets[2], joint_targets[3]);

    // gripperClosed();
    
    //TO_CONVERT PYTHON METHODS!
    // home_ch = getattr(self, bowl)
    // home_ch()

    // call[bowl2] = bowl2;
    // call[bowl2]();
    
    return 0;
    }
}

