#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <nlohmann/json.hpp>


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

cv::Mat homography_;

Arm3Link InvKin;

using json = nlohmann::json;

//function which transforms cartesian coord into polar coord
std::pair<double, double> cart2pol(double x, double y) {
  // helper, convert cartesian to polar coordinates
  double rho = std::sqrt(x * x + y * y);
  double phi = std::atan2(y, x);
  return {rho, phi};
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


        double L = calculateMeanNorm(arr) / (cos(THETA_EXT) - cos(THETA_RET));
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

        float l = (l1 + l2)/ 2;
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

// std::tuple<float, float, float> TransformCoord(float x1, float y1, float r) {
// void TransformCoord(double x1, double y1, double r){
void TransformCoord(std::vector<cv::Point2f>& points) {
    if (!homography_.empty()) {
        // cv::Mat a = (cv::Mat_<float>(1, 2) << x1, y1);
        // cv::Mat a_homogeneous;
        // cv::convertPointsToHomogeneous(a, a_homogeneous);

        // cv::Point2f point(x1, y1);
        // std::vector<cv::Point2f> points_src = point;
        std::vector<cv::Point2f> points_in, points_out;
        // cv::convertPointsToHomogeneous(points_src, points_in);

        cv::perspectiveTransform(points, points_out, homography_);
        
        // cv::Mat res;
        // cv::perspectiveTransform(a_homogeneous, res, homography_);
        
        // float x_res = res.at<float>(0, 0);
        // float y_res = res.at<float>(0, 1);
        // std::cout << x_res << " " << y_res << std::endl;
        // return std::make_tuple(x_res, y_res, DEFAULT_ROT);
    } 
    else {
        throw std::runtime_error("run or load calibration first!");
    }
}

int main(){
    loadCalibrate();
    // TransformCoord(2, 4, 3);
    return 0;
}