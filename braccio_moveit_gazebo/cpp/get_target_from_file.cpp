#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include "auto_targetter.hpp" // Assuming you have an "auto_targetter" library


int main(int argc, char** argv) {
    // auto_targetter::BraccioObjectTargetInterface auto_targ;
    ros::init(argc, argv, "get_target");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spin(1);
    spin.start();

    BraccioObjectInterface auto_targ;

    std::string target_file = "targets.txt";
    std::ifstream t(target_file);

    if (t.is_open()) {
        std::vector<std::vector<std::string>> tar_arr;

        std::string line;
        while (std::getline(t, line)) {
            std::istringstream iss(line);
            std::vector<std::string> tokens;
            std::string token;

            while (iss >> token) {
                tokens.push_back(token);
            }

            tar_arr.push_back(tokens);
        }

        t.close();

        for (size_t j = 0; j < tar_arr.size(); ++j) {
            std::string inp_ch = tar_arr[j][0];
            inp_ch.erase(std::remove(inp_ch.begin(), inp_ch.end(), '\''), inp_ch.end());
            std::cout << inp_ch << std::endl;

            auto_targ.get_link_choose(inp_ch);

            std::string bowl_ch = tar_arr[j][1];
            bowl_ch.erase(std::remove(bowl_ch.begin(), bowl_ch.end(), '\''), bowl_ch.end());
            std::cout << inp_ch << " " << bowl_ch << std::endl;
            auto_targ.loadCalibrate();

            auto_targ.goTarget(bowl_ch);
        }
    } else {
        std::cerr << "Failed to open " << target_file << std::endl;
    }

    return 0;
}
