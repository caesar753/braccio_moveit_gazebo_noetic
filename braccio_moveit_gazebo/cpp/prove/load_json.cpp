#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#include <nlohmann/json.hpp>
using json = nlohmann::json;


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

int main() {
    // try {
        std::ifstream file("calibration.json");
        if (!file.is_open()) {
            std::cerr << "calibration.json not in current directory, run calibration first" << std::endl;
            // return;
            // calibrate();
        }

        json calib = json::parse(file);
        // file >> calib;
        file.close();

        const auto s = calib.dump(); // serialize to std::string
        std::cout << "JSON string: " << s << '\n';
        std::cout << "JSON size  : " << s.size() << '\n';



        std::vector<std::vector<double>> src_pts = calib["src_pts"];
        std::vector<std::vector<double>> dst_angs = calib["dst_angs"];

        std::vector<double> src_pts_0 = src_pts[0];
        std::vector<double> src_pts_1 = src_pts[1];

        std::cout << "Size of src_pts_0 is " << src_pts_0.size()<<'\n';

        for (int i = 0; i < 7; i++){
            for (int j = 0; j < 2; j++){

                std::cout << "Output of src_pts["<< i<< "][" << j << "] is " << (std::to_string(src_pts[i][j])) << '\n';
            }
        }

        std::cout << '\n';

        for (int i = 0; i < 2; i++){
             std::cout << "Output of src_pts_0[" << i << "] is" << (std::to_string(src_pts_0[i])) << '\n';
        }



    //     std::vector<double> s_ret_pts, s_ext_pts;
    //     for (int i = 1; i < src_pts.size(); i += 2) {
    //         s_ret_pts.push_back(src_pts[i][0]);
    //         s_ret_pts.push_back(src_pts[i][1]);
    //         s_ext_pts.push_back(src_pts[i + 1][0]);
    //         s_ext_pts.push_back(src_pts[i + 1][1]);
    //     }

    //     std::vector<double> arr(s_ret_pts.size());
    //     for (int i = 0; i < s_ret_pts.size(); i++) {
    //         arr[i] = s_ret_pts[i] - s_ext_pts[i];
    //     }

    //     // Define THETA_EXT and THETA_RET
    //     // double THETA_EXT = 0.0;  // Replace with the actual values
    //     // double THETA_RET = 0.0;  // Replace with the actual values

    //     double L = cv::norm(arr) / s_ret_pts.size() / (cos(THETA_EXT) - cos(THETA_RET));

    //     for (int i = 0; i < s_ret_pts.size(); i += 2) {
    //         arr[0] = s_ret_pts[i] - src_pts[0][0];
    //         arr[1] = s_ret_pts[i + 1] - src_pts[0][1];
    //     }

    //     double l1 = cv::norm(arr) / (s_ret_pts.size() / 2) - L * cos(THETA_RET);

    //     for (int i = 0; i < s_ext_pts.size(); i += 2) {
    //         arr[0] = s_ext_pts[i] - src_pts[0][0];
    //         arr[1] = s_ext_pts[i + 1] - src_pts[0][1];
    //     }

    //     double l2 = cv::norm(arr) / (s_ext_pts.size() / 2) - L * cos(THETA_EXT);

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

    //     cv::Mat h = cv::findHomography(src_pts_mat, dst_pts_mat);
    //     homography_ = h;
    //     // Assuming 'InvKin' and 'Arm3Link' classes are defined elsewhere
    //     std::vector<double> arm_vect = {L / 2, L / 2, l + L_FUDGE};
    //     InvKin = arm_vect;

    //     std::cout << "calibration loaded." << std::endl;
    //     std::cout << "estimated l = " << l << std::endl;
    //     std::cout << "estimated L = " << L << std::endl;

    //     cv::destroyAllWindows();
    // } catch (const std::exception& e) {
    //     std::cerr << "Error: " << e.what() << std::endl;
    //     std::cerr << "calibration.json not in current directory, run calibration first" << std::endl;
    // }

    return 0;
}

