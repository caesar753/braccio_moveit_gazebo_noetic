#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <opencv2/opencv.hpp>
// #include <opencv/highgui/highgui.hpp>
#include <jsoncpp/json/json.h>
#include <cmath>
// #include <InvKin.hpp>

#define THETA_EXT 0.27
#define THETA_RET M_PI/4

#define L_FUDGE 0.08

#define Z_MAX_SIDE -0.03
#define Z_MAX_DOWN 0
#define Z_MIN -0.045

#define CLOSE_ENOUGH 0.02
#define DEFAULT_ROT 0

#define S_SIDE_MAX 0.4
#define S_SIDE_MIN 0.161
#define S_TOP_MAX 0.29

using namespace std;

typedef struct {
    double x;
    double y;
    double r;
} LinkPosition;

double L = 0;
double l = 0;

double THETA_RET_RAD = 0;
double THETA_EXT_RAD = 0;

double X_MIN = 0;
double X_MAX = 0;
double Y_MIN = 0;
double Y_MAX = 0;

int linkStatesCount = 0;
gazebo_msgs::LinkStates linkStatesData;

void linkStateCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    linkStatesCount = msg->name.size();
    linkStatesData = *msg;
}

void transform(double x1, double y1, double& x2, double& y2, double h[9])
{
    cv::Mat src = (cv::Mat_<double>(3,1) << x1, y1, 1);
    cv::Mat dst = cv::Mat::zeros(3,1,CV_64F);
    cv::Mat H = cv::Mat(3,3,CV_64F,h);
    dst = H * src;
    x2 = dst.at<double>(0,0) / dst.at<double>(2,0);
    y2 = dst.at<double>(1,0) / dst.at<double>(2,0);
}

void cart2pol(double x, double y, double& rho, double& phi)
{
    rho = sqrt(x*x + y*y);
    phi = atan2(y, x);
}

void pol2cart(double rho, double phi, double& x, double& y)
{
    x = rho * cos(phi);
    y = rho * sin(phi);
}

double getOtherAngles(double theta_shoulder, double& theta_wrist, double& theta_elbow)
{
    theta_wrist = theta_shoulder + M_PI/2;
    theta_elbow = M_PI/2 - 2*theta_shoulder;
}

// class BraccioObjectTargetInterface
// {
// private:
//     ros::NodeHandle nh_;
//     moveit::planning_interface::MoveGroupInterface move_group_;
//     moveit::planning_interface::MoveGroupInterface gripper_group_;
//     InvKin kinematics_;
//     ros::Subscriber linkStateSub_;
//     std::string linkChoose_;
//     cv::Mat homography_;

// public:
//     BraccioObjectTargetInterface() : nh_(""), move_group_("braccio_arm"), gripper_group_("braccio_gripper")
//     {
//         moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//         THETA_RET_RAD = THETA_RET;
//         THETA_EXT_RAD = THETA_EXT;

//         linkStateSub_ = nh_.subscribe("/gazebo/link_states", 10, link

class BraccioObjectTargetInterface {
private:

    // Define the structures for holding link states and positions
    struct LinkState {
        std::string name;
        geometry_msgs::Pose pose;
    };
    
    struct LinkStates {
        std::vector<LinkState> link_states;
    };
    
    // Define the Arm3Link class for inverse kinematics calculations
    class Arm3Link {
        // Implementation details for inverse kinematics
    };

    ros::NodeHandle nh;
    ros::Subscriber states_sub;

    std::string link_choose;
    LinkStates linkstate_data;

    Arm3Link kinematics;
    cv::Mat homography;
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface gripper_group;

public:
    BraccioObjectTargetInterface() : nh("~"), move_group("braccio_arm"), gripper_group("braccio_gripper") {
        // Initialize ROS
        moveit_commander::roscpp_initialize(sys.argv);
        ros::init(argc, argv, "braccio_xy_bb_target");

        states_sub = nh.subscribe("/gazebo/link_states", 1, &BraccioObjectTargetInterface::linkstate_callback, this);
    }

    void linkstate_callback(const LinkStates::ConstPtr& data) {
        linkstate_data = *data;
    }

    std::tuple<float, float, float> transform(float x1, float y1, float r) {
        if (homography.empty()) {
            throw std::runtime_error("Run or load calibration first!");
        }

        cv::Mat a(1, 1, CV_32FC2);
        cv::Mat res;
        a.at<cv::Point2f>(0, 0) = cv::Point2f(x1, y1);
        cv::perspectiveTransform(a, res, homography);
        float res_x = res.at<float>(0, 0);
        float res_y = res.at<float>(0, 1);
        return std::make_tuple(res_x, res_y, DEFAULT_ROT);
    }

    std::tuple<float, float, float> get_box_position() {
        float x, y, r;
        std::tie(x, y, r) = get_link_position({link_choose});
        return transform(x, y, r);
    }

    void get_link_choose(const std::string& lk) {
        link_choose = lk;
        std::cout << lk << std::endl;
        std::cout << link_choose << std::endl;
    }

    std::tuple<float, float, float> get_link_position(const std::vector<std::string>& link_names) {
        float x = 0.0;
        float y = 0.0;
        int n = 0;
        for (const auto& l : link_names) {
            auto it = std::find_if(link