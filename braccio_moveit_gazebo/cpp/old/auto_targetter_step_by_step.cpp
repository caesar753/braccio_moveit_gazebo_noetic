#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "InvKin.hpp"

#define THETA_EXT 0.27
#define THETA_RET M_PI / 4

#define L_FUDGE 0.08

#define Z_MAX_SIDE -0.03
#define Z_MAX_DOWN 0
#define Z_MIN -0.045

#define CLOSE_ENOUGH 0.02
#define DEFAULT_ROT 0

#define S_SIDE_MAX 0.4
#define S_SIDE_MIN 0.161
#define S_TOP_MAX 0.29




std::tuple<double, double> cart2pol(double x, double y) {
  double rho = std::sqrt(x * x + y * y);
  double phi = std::atan2(y, x);
  return std::make_tuple(rho, phi);
}

std::tuple<double, double> pol2cart(double rho, double phi) {
  double x = rho * std::cos(phi);
  double y = rho * std::sin(phi);
  return std::make_tuple(x, y);
}

std::tuple<double, double> get_other_angles(double theta_shoulder) {
  double theta_wrist = theta_shoulder + M_PI / 2;
  double theta_elbow = M_PI / 2 - 2 * theta_shoulder;
  return std::make_tuple(theta_wrist, theta_elbow);
}


class BraccioObjectTargetInterface
{

private:
  ros::NodeHandle nh;
  moveit::planning_interface::MoveGroupInterface* move_group;
  moveit::planning_interface::MoveGroupInterface* gripper_group;
  gazebo_msgs::LinkStates::ConstPtr linkstate_data;
  std::string link_choose;
  cv::Mat homography;
  Arm3Link kinematics;
  ros::Subscriber states_sub;


public:
  BraccioObjectTargetInterface() : link_choose("") {
    moveit_commander::roscpp_initialize(sys.argv);
    ros::init(argc, argv, "braccio_xy_bb_target");

    std::string group_name = "braccio_arm";
    move_group = new moveit::planning_interface::MoveGroupInterface(group_name);
    gripper_group = new moveit::planning_interface::MoveGroupInterface("braccio_gripper");

    // homography = nullptr;
     cv::Mat homography;
     Arm3Link kinematics;

    // kinematics = new InvKin::Arm3Link();
    states_sub = nh.subscribe("/gazebo/link_states", 1, &BraccioObjectTargetInterface::linkstate_callback, this);
  }

  void linkstate_callback(const gazebo_msgs::LinkStates::ConstPtr& data) {
    linkstate_data = data;
  }

  void transform(double x1, double y1, double r, double& x, double& y, double& rotation) {
    if (homography != nullptr) {
      cv::Mat src(1, 1, CV_32FC2);
      cv::Mat dst(1, 1, CV_32FC2);
      src.at<cv::Vec2f>(0, 0) = cv::Vec2f(x1, y1);
      cv::perspectiveTransform(src, dst, *homography);
      x = dst.at<cv::Vec2f>(0, 0)[0];
      y = dst.at<cv::Vec2f>(0, 0)[1];
      rotation = DEFAULT_ROT;
    } else {
      throw std::runtime_error("Run or load calibration first!");
    }
  }

  std::vector<double> get_box_position() {
    std::vector<double> pose = get_link_position({link_choose});
    double x, y, rotation;
    transform(pose[0], pose[1], pose[2], x, y, rotation);
    std::vector<double> transformed_pose = {x, y, rotation};
    return transformed_pose;
  }

  void get_link_choose(const std::string& lk) {
    link_choose = lk;
    std::cout << lk << std::endl;
    std::cout << link_choose << std::endl;
  }

  std::vector<double> get_link_position(const std::vector<std::string>& link_names) {
    double x = 0.0;
    double y = 0.0;
    int n = 0;
    for (const std::string& l : link_names) {
      auto it = std::find(linkstate_data->name.begin(), linkstate_data->name.end(), l);
      if (it != linkstate_data->name.end()) {
        size_t ind = std::distance(linkstate_data->name.begin(), it);
        const geometry_msgs::Pose& pose = linkstate_data->pose[ind];
        x += pose.position.x;
        y += pose.position.y;
        ++n;
      }
    }
    std::vector<double> result = {x / n, y / n, DEFAULT_ROT};
    return result;
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "braccio_node");

  BraccioObjectTargetInterface braccio;

  // Other code and operations

  ros::spin();

  return 0;
}