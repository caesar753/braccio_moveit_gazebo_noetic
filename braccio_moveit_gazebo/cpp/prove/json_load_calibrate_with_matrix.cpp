#include <iostream>
#include <fstream>
#include <vector>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

const double THETA_EXT = 0.27;
const double THETA_RET = M_PI / 4.0;

using json = nlohmann::json;

std::pair<double, double> pol2cart(double rho, double phi) {
  // helper, convert polar to cartesian coordinates
  double x = rho * std::cos(phi);
  double y = rho * std::sin(phi);
  return std::make_pair(x, y);
}

void loadCalibrate() {
    try {
        json calib;
        std::ifstream file("calibration.json");

        if (!file.is_open()) {
            std::cerr << "Failed to open calibration.json" << std::endl;
            return;
        }

        // file >> calib;
        // file.close();
        calib = json::parse(file);
        file.close();

        const json& src_pts = calib["src_pts"];
        const json& dst_angs = calib["dst_angs"];

        Eigen::MatrixXd src_pts_matrix(src_pts.size(), 2);
        Eigen::MatrixXd dst_pts_matrix(dst_angs.size() + 1, 2);

        for (int i = 0; i < src_pts.size(); ++i) {
            src_pts_matrix(i, 0) = src_pts[i][0].get<double>();
            src_pts_matrix(i, 1) = src_pts[i][1].get<double>();
        }

        Eigen::MatrixXd s_ret_pts = src_pts_matrix.block(1, 0, src_pts_matrix.rows() - 1, src_pts_matrix.cols());
        Eigen::MatrixXd s_ext_pts = src_pts_matrix.block(2, 0, src_pts_matrix.rows() - 2, src_pts_matrix.cols());

        Eigen::MatrixXd arr = s_ret_pts - s_ext_pts;
        double L = sqrt(arr.array().square().rowwise().sum().mean()) / (cos(THETA_EXT) - cos(THETA_RET));

        Eigen::MatrixXd arr1 = s_ret_pts - src_pts_matrix.row(0).transpose();
        double l1 = sqrt(arr1.array().square().rowwise().sum().mean()) - L * cos(THETA_RET);
        Eigen::MatrixXd arr2 = s_ext_pts - src_pts_matrix.row(0).transpose();
        double l2 = sqrt(arr2.array().square().rowwise().sum().mean()) - L * cos(THETA_EXT);
        double l = (l1 + l2) / 2;

        dst_pts_matrix(0, 0) = 0;
        dst_pts_matrix(0, 1) = 0;

        for (int i = 0; i < dst_angs.size(); ++i) {
            double phi = dst_angs[i][0].get<double>();
            double rho = L * cos(dst_angs[i][1].get<double>()) + l;
            double x = rho * cos(phi);
            double y = rho * sin(phi);
            dst_pts_matrix(i + 1, 0) = x;
            dst_pts_matrix(i + 1, 1) = y;
        }

        cv::Mat homography_;
        cv::Mat H = cv::findHomography(src_pts_matrix, dst_pts_matrix, cv::RANSAC, 5);

        // Store the homography matrix for further use
        homography_ = H;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}


int main() {
    loadCalibrate();
    // You can now use calibration.homography for further processing
    return 0;
}
