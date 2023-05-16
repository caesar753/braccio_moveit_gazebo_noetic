#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <opencv2/opencv.hpp>
#include <json/json.h>
#include <cmath>
#include "InvKin.hpp"

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

class BraccioObjectTargetInterface
{
private:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::MoveGroupInterface gripper_group_;
    InvKin kinematics_;
    ros::Subscriber linkStateSub_;
    std::string linkChoose_;
    cv::Mat homography_;

public:
    BraccioObjectTargetInterface() : nh_(""), move_group_("braccio_arm"), gripper_group_("braccio_gripper")
    {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        THETA_RET_RAD = THETA_RET;
        THETA_EXT_RAD = THETA_EXT;

        linkStateSub_ = nh_.subscribe("/gazebo/link_states", 10, link
