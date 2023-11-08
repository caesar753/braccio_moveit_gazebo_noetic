#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>

class Arm3Link {
public:
    Arm3Link(std::vector<double> L = {1.0, 1.0, 0.8}) {
        q = {0.0, 0.0, 0.0};
        this->L = L;
        max_y = 1.0;
        min_y = 0.0;
        end_angle_tol = 0.05;
        end_angle = -M_PI / 2.0;
        max_angles = {1.6, M_PI / 2.0, M_PI / 2.0};
        min_angles = {0.27, -M_PI / 2.0, -M_PI / 2.0};
    }

    std::vector<double> get_xy(const std::vector<double>& q) {
        double x = L[0] * cos(q[0]) + L[1] * cos(q[0] + q[1]) + L[2] * cos(std::accumulate(q.begin(), q.end(), 0.0));
        double y = L[0] * sin(q[0]) + L[1] * sin(q[0] + q[1]) + L[2] * sin(std::accumulate(q.begin(), q.end(), 0.0));
        return {x, y};
    }

    std::vector<double> inv_kin(double x, double min_y, double max_y, double end_angle) {
        this->min_y = min_y;
        this->max_y = max_y;
        this->end_angle = end_angle;

        auto distance_to_default = [this, x](const std::vector<double>& q) {
            double result = (L[0] * cos(q[0]) + L[1] * cos(q[0] + q[1]) + L[2] * cos(std::accumulate(q.begin(), q.end(), 0.0))) - x;
            return result * result;
        };

        auto y_upper_constraint = [this](const std::vector<double>& q) {
            double y = (L[0] * sin(q[0]) + L[1] * sin(q[0] + q[1]) + L[2] * sin(std::accumulate(q.begin(), q.end(), 0.0)));
            return max_y - y;
        };

        auto y_lower_constraint = [this](const std::vector<double>& q) {
            double y = (L[0] * sin(q[0]) + L[1] * sin(q[0] + q[1]) + L[2] * sin(std::accumulate(q.begin(), q.end(), 0.0)));
            return y - min_y;
        };

        auto joint_limits_upper_constraint = [this](const std::vector<double>& q) {
            std::vector<double> result;
            for (size_t i = 0; i < q.size(); ++i) {
                result.push_back(max_angles[i] - q[i]);
            }
            return result;
        };

        auto joint_limits_lower_constraint = [this](const std::vector<double>& q) {
            std::vector<double> result;
            for (size_t i = 0; i < q.size(); ++i) {
                result.push_back(q[i] - min_angles[i]);
            }
            return result;
        };

        auto joint_limits_last_orientation = [this](const std::vector<double>& q) {
            return end_angle_tol - std::abs(std::accumulate(q.begin(), q.end(), 0.0) - end_angle);
        };

        q = optimize(distance_to_default, q, {joint_limits_last_orientation, joint_limits_upper_constraint, joint_limits_lower_constraint, y_upper_constraint, y_lower_constraint});

        this->q = q;
        return q;
    }

private:
    std::vector<double> q;
    std::vector<double> L;
    double max_y;
    double min_y;
    double end_angle_tol;
    double end_angle;
    std::vector<double> max_angles;
    std::vector<double> min_angles;

    std::vector<double> optimize(const std::function<double(const std::vector<double>&)>& func, std::vector<double> q, const std::vector<std::function<double(const std::vector<double>&)>>& constraints) {
        // Implement your optimization method here (e.g., using a library or your own implementation).
        // This function should return the optimized q values.
        // You can use external optimization libraries like Ceres Solver or implement your own optimization algorithm.
        // For simplicity, I'm not providing the optimization code here, but you can add your preferred optimization method.
        return q;
    }
};