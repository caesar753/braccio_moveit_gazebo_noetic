#include <ros/ros.h>
#include <custom_msgs/matrix.h>

class BraccioObjectTargetInterface {
public:
    BraccioObjectTargetInterface() {
        nh = new ros::NodeHandle();
        i = 0;
        target_matrix = nh->subscribe("/targets", 1, &BraccioObjectTargetInterface::callbackMatrix, this);
        // Sleep to allow ROS to get the robot state
        ros::Duration(1.0).sleep();
        // Unregister the targets subscriber
        target_matrix.shutdown();
    }

    void callbackMatrix(const custom_msgs::matrix::ConstPtr& msg) {
        for (size_t i = 0; i < msg->targets.size(); ++i) {
            targets_list.push_back(msg->targets[i]);
            this->i = i;
        }
    }

    std::pair<int, std::vector<custom_msgs::matrix>> returnTargets() {
        return std::make_pair(i, targets_list);
    }

private:
    ros::NodeHandle* nh;
    ros::Subscriber target_matrix;
    // std::vector<custom_msgs::matrix> targets_list;
    std::vector<std::vector<std::string>> targets_list;
    int i;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "braccio_xy_bb_target");
    BraccioObjectTargetInterface braccio_interface;
    ros::spin(); // Keep the node alive

    return 0;
}
