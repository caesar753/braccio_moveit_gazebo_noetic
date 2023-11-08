#include <iostream>
#include <fstream>
#include <vector>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>

const double THETA_EXT = 0.27;
const double THETA_RET = M_PI / 4.0;

class CalibrationLoader {
public:
    CalibrationLoader() : L(0.0) {
        loadCalibrate();
    }

    void loadCalibrate() {
        try {
            Json::Value calib;
            std::ifstream file("calibration.json");

            if (!file.is_open()) {
                std::cerr << "Failed to open calibration.json" << std::endl;
                return;
            }

            file >> calib;
            // calib = Json::O(file);

            file.close();

            const Json::Value src_pts = calib["src_pts"];
            const Json::Value dst_angs = calib["dst_angs"];

            std::vector<std::vector<double>> src_pts_matrix;
            // std::vector<std::vector<double>> dst_pts_matrix;

            for (int i = 0; i < src_pts.size(); ++i) {
                std::vector<double> point;
                point.push_back(src_pts[i][0].asDouble());
                point.push_back(src_pts[i][1].asDouble());
                src_pts_matrix.push_back(point);
            }

            std::vector<std::vector<double>> s_ret_pts(src_pts_matrix.begin() + 1, src_pts_matrix.end());
            std::vector<std::vector<double>> s_ext_pts(src_pts_matrix.begin() + 2, src_pts_matrix.end());

            std::vector<std::vector<double>> arr;
            for (size_t i = 0; i < s_ret_pts.size(); ++i) {
                std::vector<double> diff;
                for (size_t j = 0; j < s_ret_pts[i].size(); ++j) {
                    diff.push_back(s_ret_pts[i][j] - s_ext_pts[i][j]);
                }
                arr.push_back(diff);
            }

            L = calculateL(arr);

            std::vector<std::vector<double>> arr1;
            for (size_t i = 0; i < s_ret_pts.size(); ++i) {
                std::vector<double> diff;
                for (size_t j = 0; j < s_ret_pts[i].size(); ++j) {
                    diff.push_back(s_ret_pts[i][j] - src_pts_matrix[0][j]);
                }
                arr1.push_back(diff);
            }

            double l1 = calculateLength(arr1) - L * cos(THETA_RET);

            std::vector<std::vector<double>> arr2;
            for (size_t i = 0; i < s_ext_pts.size(); ++i) {
                std::vector<double> diff;
                for (size_t j = 0; j < s_ext_pts[i].size(); ++j) {
                    diff.push_back(s_ext_pts[i][j] - src_pts_matrix[0][j]);
                }
                arr2.push_back(diff);
            }

            double l2 = calculateLength(arr2) - L * cos(THETA_EXT);

            double l = (l1 + l2) / 2.0;

            std::vector<std::vector<double>> dst_pts_matrix(dst_angs.size() + 1, std::vector<double>(2, 0.0));

            for (int i = 0; i < dst_angs.size(); ++i) {
                double phi = dst_angs[i][0].asDouble();
                double rho = L * cos(dst_angs[i][1].asDouble()) + l;
                double x = rho * cos(phi);
                double y = rho * sin(phi);
                dst_pts_matrix[i + 1][0] = x;
                dst_pts_matrix[i + 1][1] = y;
            }

            cv::Mat H = cv::findHomography(convertToCvMat(src_pts_matrix), convertToCvMat(dst_pts_matrix), cv::RANSAC);

            // Store the homography matrix for further use
            this->homography = H;
        } catch (const std::exception &e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }

private:
    double L;
    cv::Mat homography;

    // Helper function to calculate the length of a vector of vectors
    double calculateLength(const std::vector<std::vector<double>> &vectors) {
        double sum = 0.0;
        for (const auto &vec : vectors) {
            double squaredSum = 0.0;
            for (double val : vec) {
                squaredSum += val * val;
            }
            sum += sqrt(squaredSum);
        }
        return sum / vectors.size();
    }

    // Helper function to calculate L
    double calculateL(const std::vector<std::vector<double>> &arr) {
        double sum = 0.0;
        for (const auto &vec : arr) {
            double squaredSum = 0.0;
            for (double val : vec) {
                squaredSum += val * val;
            }
            sum += sqrt(squaredSum);
        }
        return sum / (cos(THETA_EXT) - cos(THETA_RET));
    }

    // Helper function to convert a vector of vectors to a cv::Mat
    cv::Mat convertToCvMat(const std::vector<std::vector<double>> &vectors) {
        cv::Mat mat(vectors.size(), vectors[0].size(), CV_64F);
        for (int i = 0; i < vectors.size(); ++i) {
            for (int j = 0; j < vectors[0].size(); ++j) {
                mat.at<double>(i, j) = vectors[i][j];
            }
        }
        return mat;
    }
};

int main() {
    CalibrationLoader calibration;
    // You can now use calibration.homography for further processing
    calibration.loadCalibrate();
    return 0;
}