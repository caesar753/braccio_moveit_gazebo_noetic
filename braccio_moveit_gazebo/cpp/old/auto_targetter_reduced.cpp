// #include <iostream>
// #include <vector>
// #include <fstream>
// #include <opencv2/opencv.hpp>
// #include <jsoncpp/json/json.h>

// #include <ros/ros.h>
// #include <moveit_commander/move_group_interface.h>
// #include <geometry_msgs/Pose.h>
// #include <gazebo_msgs/LinkStates.h>
// #include <gazebo_msgs/ModelState.h>
// #include <gazebo_msgs/SetModelState.h>

// #define THETA_EXT 0.27
// #define THETA_RET M_PI/4

// #define L_FUDGE 0.08

// #define Z_MAX_SIDE -0.03
// #define Z_MAX_DOWN 0
// #define Z_MIN -0.045

// #define CLOSE_ENOUGH 0.02
// #define DEFAULT_ROT 0

// #define S_SIDE_MAX 0.4
// #define S_SIDE_MIN 0.161
// #define S_TOP_MAX 0.29

// cv::Mat homography;
// InvKin kinematics;
// gazebo_msgs::LinkStates linkstateData;
// std::string linkChoose;

// std::vector<double> getJointGoal(const moveit::planning_interface::MoveGroupInterface& moveGroup) {
//   std::vector<double> jointGoal = moveGroup.getCurrentJointValues();
//   return jointGoal;
// }

// void go_to_j(moveit::planning_interface::MoveGroupInterface& moveGroup, double j0, double j1, double j2, double j3) {
//   std::vector<double> jointGoal = getJointGoal(moveGroup);
//   if (j0 != 0) {
//     jointGoal[0] = j0;
//   }
//   if (j1 != 0) {
//     jointGoal[1] = j1;
//   }
//   if (j2 != 0) {
//     jointGoal[2] = j2;
//   }
//   if (j3 != 0) {
//     jointGoal[3] = j3;
//   }
//   moveGroup.go(jointGoal, true);
//   moveGroup.stop();
// }

// void go_to_joint(moveit::planning_interface::MoveGroupInterface& moveGroup, const std::vector<double>& jointTargets) {
//   std::vector<double> jointGoal = getJointGoal(moveGroup);
//   jointGoal[0] = jointTargets[0];
//   jointGoal[1] = jointTargets[1];
//   jointGoal[2] = jointTargets[2];
//   jointGoal[3] = jointTargets[3];
//   jointGoal[4] = 1.5708;
//   moveGroup.go(jointGoal, true);
//   moveGroup.stop();
// }

// void gripper_close(moveit::planning_interface::MoveGroupInterface& gripperGroup) {
//   std::vector<double> jointGoal = gripperGroup.getCurrentJointValues();
//   jointGoal[0] = 1.2;
//   jointGoal[1] = 1.2;
//   gripperGroup.go(jointGoal, true);
//   gripperGroup.stop();
// }

// void go_to_xy(double x, double y, double r, std::string how, std::string bowl) {
//   if (how == "top") {
//     double s, phi;
//     std::tie(s, phi) = cart2pol(x, y);
//     std::cout << s << ", " << phi << std::endl;
//     std::vector<double> q = kinematics.inv_kin(s, Z_MIN, Z_MAX_DOWN, -M_PI/2);
//     std::vector<double> xy = kinematics.get_xy(q);
//     if (std::abs(xy[0] - s) > CLOSE_ENOUGH) {
//       std::cout << "NO SOLUTION FOUND" << std::endl;
//       std::cout << "goal distance = " << s << std::endl;
//       std::cout << "closest solution = " << xy[0] << std::endl;
//       return;
//     }
//   }
// }

// void go_to_home_0(moveit::planning_interface::MoveGroupInterface& moveGroup, moveit::planning_interface::MoveGroupInterface& gripperGroup) {
//   go_to_pick(moveGroup);
//   go_to_j(moveGroup, 2.355);
//   go_to_j(moveGroup, 1.67, 0.10, 0.5);
//   gripper_open(gripperGroup);
//   gripper_open(gripperGroup);
// }

// void go_to_home_1(moveit::planning_interface::MoveGroupInterface& moveGroup, moveit::planning_interface::MoveGroupInterface& gripperGroup) {
//   go_to_pick(moveGroup);
//   go_to_j(moveGroup, 0.785);
//   go_to_j(moveGroup, 1.57, 3.00, 2.55);
//   gripper_open(gripperGroup);
//   gripper_open(gripperGroup);
// }

// void linkch(std::vector<std::string>& linkChoose) {
//   std::ifstream groupsFile("choosen.txt");
//   std::vector<std::vector<std::string>> groups;
//   std::string line;
//   while (std::getline(groupsFile, line)) {
//     std::istringstream iss(line);
//     std::vector<std::string> tokens{std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>()};
//     groups.push_back(tokens);
//   }
  
//   std::ifstream posFile("posizioni_mm.txt");
//   std::vector<std::vector<std::string>> posizioni;
//   while (std::getline(posFile, line)) {
//     std::istringstream iss(line);
//     std::vector<std::string> tokens{std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>()};
//     posizioni.push_back(tokens);
//   }
  
//   for (const auto& posizione : posizioni) {
//     if (std::find(groups.begin(), groups.end(), posizione[0]) != groups.end()) {
//       linkChoose.push_back(posizione[4]);
//     }
//   }
// }

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "braccio_xy_bb_target");
//   ros::NodeHandle nh;

//   moveit_commander::MoveGroupInterface moveGroup("braccio_arm");
//   moveit_commander::MoveGroupInterface gripperGroup("braccio_gripper");

//   kinematics = InvKin::Arm3Link();

//   ros::Subscriber linkstateSub = nh.subscribe("/gazebo/link_states", 10, linkstate_callback);

//   // Load calibration
//   std::ifstream calibFile("calibration.json");
//   Json::Value calib;
//   calibFile >> calib;
//   cv::Mat srcPts(calib["src_pts"].size(), 2, CV_32FC1);
//   cv::Mat dstPts(calib["dst_pts"].size(), 2, CV_32FC1);
//   for (int i = 0; i < calib["src_pts"].size(); i++) {
//     srcPts.at<float>(i, 0) = calib["src_pts"][i][0].asFloat();
//     srcPts.at<float>(i, 1) = calib["src_pts"][i][1].asFloat();
//     dstPts.at<float>(i, 0) = calib["dst_pts"][i][0].asFloat();
//     dstPts.at<float>(i, 1) = calib["dst_pts"][i][1].asFloat
//   }
//   homography = cv::findHomography(srcPts, dstPts);

//   ros::Rate loop_rate(10);

//   while (ros::ok()) {
//     ros::spinOnce();
//     loop_rate.sleep();
//   }

//   return 0;
// }


//SECOND VERSION

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <jsoncpp/json/json.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include "InvKin.hpp"


typedef struct {
    double x;
    double y;
    double r;
} LinkPosition;


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

cv::Mat homography;
InvKin::Arm3Link kinematics;
ros::Subscriber linkstateSub;

int linkStatesCount = 0;
gazebo_msgs::LinkStates linkStatesData;

void linkstate_callback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
  // callback to get link location for cube from gazebo
//   linkstate_data = *data;
    linkStatesCount = msg->name.size();
    linkStatesData = *msg;
}

void transform(double x1, double y1, double r, double& x, double& y, double& rotation) {
  // transform from gazebo coordinates into braccio coordinates
  if (!homography.empty()) {
    cv::Mat a(1, 1, CV_32FC2);
    a.at<cv::Vec2f>(0, 0)[0] = x1;
    a.at<cv::Vec2f>(0, 0)[1] = y1;
    cv::Mat res;
    cv::perspectiveTransform(a, res, homography);
    x = res.at<cv::Vec2f>(0, 0)[0];
    y = res.at<cv::Vec2f>(0, 0)[1];
    rotation = DEFAULT_ROT;
  } else {
    throw std::invalid_argument("Run or load calibration first!");
  }
}

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

class BraccioObjectTargetInterface {
public:
  BraccioObjectTargetInterface() {
    moveit_commander::roscpp_initialize(argc, argv);
    ros::init(argc, argv, "braccio_xy_bb_target");
    ros::NodeHandle nh;

    moveit_commander::MoveGroupInterface moveGroup("braccio_arm");
    moveit_commander::MoveGroupInterface gripperGroup("braccio_gripper");

    homography = cv::Mat();
    kinematics = InvKin::Arm3Link();

    linkstateSub = nh.subscribe("/gazebo/link_states", 1, linkstate_callback);
  }

  void load_calibrate() {
    // load mapping points from gazebo to robot frame, estimate l and L, generate homography map
    std::ifstream file("calibration.json");
    if (file.is_open()) {
      Json::Reader reader;
      Json::Value calib;
      if (reader.parse(file, calib)) {
        const Json::Value& src_pts_json = calib["src_pts"];
        const Json::Value& dst_angs_json = calib["dst_angs"];

        std::vector<cv::Point2f> src_pts, dst_pts;

        for (const auto& pt_json : src_pts_json) {
          double x = pt_json[0].asDouble();
          double y = pt_json[1].asDouble();
          src_pts.push_back(cv::Point2f(x, y));
        }

        for (const auto& ang_json : dst_angs_json) {
          double phi = ang_json[0].asDouble();
          double rho = L * std::cos(ang_json[1].asDouble()) + l;
          double x, y;
          std::tie(x, y) = pol2cart(rho, phi);
          dst_pts.push_back(cv::Point2f(x, y));
        }

        homography = cv::findHomography(src_pts, dst_pts);

        kinematics = InvKin::Arm3Link(L / 2, L / 2, l + L_FUDGE);

        std::cout << "Calibration loaded." << std::endl;
        std::cout << "Estimated l = " << l << std::endl;
        std::cout << "Estimated L = " << L << std::endl;
      } else {
        std::cerr << "Failed to parse calibration JSON." << std::endl;
      }

      file.close();
    } else {
      std::cerr << "calibration.json not found. Run calibration first." << std::endl;
    }
  }

  void get_link_choose(const std::string& lk) {
    link_choose = lk;
    std::cout << lk << std::endl;
    std::cout << link_choose << std::endl;
  }

  std::vector<std::string> get_link_position(const std::vector<std::string>& link_names) {
    // get mean position of a list of links
    double x = 0.0;
    double y = 0.0;
    int n = 0;
    std::vector<std::string> link_positions;

    for (const std::string& l : link_names) {
      auto it = std::find(linkStatesData.name.begin(), linkStatesData.name.end(), l);
      if (it != linkStatesData.name.end()) {
        int ind = std::distance(linkStatesData.name.begin(), it);
        double res_x = linkStatesData.pose[ind].position.x;
        double res_y = linkStatesData.pose[ind].position.y;
        x += res_x;
        y += res_y;
        n++;
        link_positions.push_back(l);
      }
    }

    x /= n;
    y /= n;

    return link_positions;
  }

  void go_to_joint(double j0, double j1, double j2, double j3) {
    // update arm joints
    std::vector<double> joint_goal = move_group.getCurrentJointValues();
    if (j0 != -1.0)
      joint_goal[0] = j0;
    if (j1 != -1.0)
      joint_goal[1] = j1;
    if (j2 != -1.0)
      joint_goal[2] = j2;
    if (j3 != -1.0)
      joint_goal[3] = j3;
    go_to_joint(joint_goal);
  }

  void go_to_joint(const std::vector<double>& joint_targets) {
    std::vector<double> joint_goal = move_group.getCurrentJointValues();
    joint_goal[0] = joint_targets[0];
    joint_goal[1] = joint_targets[1];
    joint_goal[2] = joint_targets[2];
    joint_goal[3] = joint_targets[3];
    joint_goal[4] = 1.5708;
    move_group.go(joint_goal);
    move_group.stop();
  }

  void gripper_close() {
    go_gripper(1.2);
  }

  std::tuple<double, double> get_down_targets(double x, double y) {
    double s, phi;
    std::tie(s, phi) = cart2pol(x, y);
    std::vector<double> q = kinematics.inv_kin(s, Z_MIN, Z_MAX_DOWN, -M_PI / 2);
    std::tuple<double, double> xy = kinematics.get_xy(q);

    if (std::abs(std::get<0>(xy) - s) > CLOSE_ENOUGH) {
      std::cout << "NO SOLUTION FOUND" << std::endl;
      std::cout << "goal distance = " << s << std::endl;
      std::cout << "closest solution = " << std::get<0>(xy) << std::endl;
      return std::make_tuple(s, std::numeric_limits<double>::quiet_NaN());
    }

    return std::make_tuple(s, phi);
  }

  int go_to_xy(double x, double y, double r, const std::string& how, bool bowl) {
    if (how == "top") {
      double s;
      std::vector<double> joint_targets;
      std::tie(s, joint_targets) = get_down_targets(x, y);

      if (joint_targets[0] < 0.0 || joint_targets[0] > 3.14) {
        std::cout << "++++++ Not in reachable area, aborting ++++++" << std::endl;
        return -1;
      }

      if (std::isnan(joint_targets[1])) {
        std::cout << "++++++ Not in reachable area, aborting ++++++" << std::endl;
        return -1;
      }
    }
  }

  void go_to_home_0() {
    go_to_pick();
    go_to_joint(2.355);
    go_to_joint(1.67, 0.10, 0.5);
    gripper_open();
    gripper_open();
  }

  void go_to_home_1() {
    go_to_pick();
    go_to_joint(0.785);
    go_to_joint(1.57, 3.00, 2.55);
    gripper_open();
    gripper_open();
  }

  std::vector<std::tuple<int, std::string, std::vector<int>>> linkch() {
    std::vector<std::tuple<int, std::string, std::vector<int>>> link_choose;
    std::ifstream groups_file("choosen.txt");
    std::ifstream positions_file("posizioni_mm.txt");

    std::vector<std::string> groups_lines;
    std::vector<std::string> positions_lines;

    std::string line;
    while (std::getline(groups_file, line)) {
      groups_lines.push_back(line);
    }

    while (std::getline(positions_file, line)) {
      positions_lines.push_back(line);
    }

    std::vector<std::vector<std::string>> parsed_groups;
    std::vector<std::vector<std::string>> parsed_positions;

    for (const std::string& groups_line : groups_lines) {
      std::istringstream iss(groups_line);
      std::vector<std::string> parsed_group;
      std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
                std::back_inserter(parsed_group));
      parsed_groups.push_back(parsed_group);
    }

    for (const std::string& positions_line : positions_lines) {
      std::istringstream iss(positions_line);
      std::vector<std::string> parsed_position;
      std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
                std::back_inserter(parsed_position));
      parsed_positions.push_back(parsed_position);
    }

    for (int i = 0; i < parsed_groups.size(); i++) {
      int ind = std::stoi(parsed_groups[i][0]);
      std::string link = parsed_groups[i][1];

      std::vector<int> positions;
      for (int j = 0; j < parsed_positions[i].size(); j++) {
        positions.push_back(std::stoi(parsed_positions[i][j]));
      }

      link_choose.push_back(std::make_tuple(ind, link, positions));
    }

    groups_file.close();
    positions_file.close();

    return link_choose;
  }

  void load_calibrate_cb() {
    load_calibrate();
  }

  std::vector<int> find_closer_target(const std::vector<int>& targets, int goal) {
    std::vector<int> sorted_targets = targets;
    std::sort(sorted_targets.begin(), sorted_targets.end());

    auto lower = std::lower_bound(sorted_targets.begin(), sorted_targets.end(), goal);

    if (lower == sorted_targets.begin()) {
      return {*lower, *(lower + 1)};
    } else if (lower == sorted_targets.end()) {
      return {*(lower - 1), *lower};
    } else {
      return {*(lower - 1), *lower};
    }
  }

  void find_orientation() {
    std::vector<std::string> link_names;
    std::vector<std::string> joint_names;
    for (const auto& l : linkstate_data.name) {
      link_names.push_back(l);
    }
    for (const auto& j : linkstate_data.name) {
      joint_names.push_back(j);
    }
    std::vector<std::string> gripper_link_names = get_link_position(link_names);
    std::vector<std::string> gripper_joint_names = get_link_position(joint_names);

    std::vector<int> theta_shoulder;

    for (const auto& l : gripper_link_names) {
      double x = linkstate_data.pose[ind].position.x;
      double y = linkstate_data.pose[ind].position.y;
      double z = linkstate_data.pose[ind].position.z;

      double x2 = x + r * std::cos(theta);
      double y2 = y + r * std::sin(theta);
      double z2 = z - Z_MAX_SIDE;

      double x3, y3, theta3;
      transform(x2, y2, r, x3, y3, theta3);
      theta_shoulder.push_back(theta3);
    }

    std::vector<std::tuple<int, std::string, std::vector<int>>> groups = linkch();

    for (const auto& group : groups) {
      std::vector<int> positions = std::get<2>(group);
      std::vector<int> possible_targets = find_closer_target(positions, theta_shoulder[0]);

      std::cout << "Link " << std::get<1>(group) << ": " << std::endl;
      std::cout << "Possible targets: " << possible_targets[0] << " or " << possible_targets[1] << std::endl;
    }
  }

  void execute_cb(const std_msgs::String::ConstPtr& msg) {
    std::string command = msg->data;

    std::cout << "Command received: " << command << std::endl;

    if (command == "calibrate") {
      calibrate();
    } else if (command == "load_calibrate") {
      load_calibrate_cb();
    } else if (command == "find_orientation") {
      find_orientation();
    } else {
      std::cout << "Invalid command." << std::endl;
    }
  }
} // namespace braccio_demo

int main(int argc, char** argv) {
  ros::init(argc, argv, "braccio_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  braccio_demo::BraccioDemo braccio_demo(nh);
  ros::Subscriber sub = nh.subscribe("braccio_command", 10, &braccio_demo::execute_cb, &braccio_demo);
  ros::spin();

  return 0;
}
