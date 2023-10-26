#include <iostream>
#include <vector>
#include <cmath>
#include <functional>
#include <algorithm>
#include <numeric>
#include <stdexcept>

class Arm3Link {
public:
    Arm3Link(std::vector<double> L = {1.0, 1.0, 0.8}) {
        q = {0.0, 0.0, 0.0};
        this->L = L;
        max_y = 1.0;
        min_y = 0.0;
        end_angle_tol = 0.05;
        end_angle = -M_PI / 2;
        max_angles = {1.6, M_PI / 2, M_PI / 2};
        min_angles = {0.27, -M_PI / 2, -M_PI / 2};
    }

    std::vector<double> get_xy(std::vector<double> q = {}) {
        if (q.empty()) {
            q = this->q;
        }

        double x = L[0] * cos(q[0]) + L[1] * cos(q[0] + q[1]) + L[2] * cos(q[0] + q[1] + q[2]);
        double y = L[0] * sin(q[0]) + L[1] * sin(q[0] + q[1]) + L[2] * sin(q[0] + q[1] + q[2]);

        return {x, y};
    }

    std::vector<double> inv_kin(std::vector<double> x, double min_y, double max_y, double end_angle = -M_PI / 2) {
        this->min_y = min_y;
        this->max_y = max_y;
        this->end_angle = end_angle;

        auto distance_to_default = [&](const std::vector<double>& q) {
            std::vector<double> x_current = get_xy(q);
            double dist = std::inner_product(x_current.begin(), x_current.end(), x.begin(), 0.0,
                std::plus<>(), [](double a, double b) { return (a - b) * (a - b); });
            return dist;
        };

        auto y_upper_constraint = [&](const std::vector<double>& q) {
            std::vector<double> xy = get_xy(q);
            return max_y - xy[1];
        };

        auto y_lower_constraint = [&](const std::vector<double>& q) {
            std::vector<double> xy = get_xy(q);
            return xy[1] - min_y;
        };

        auto joint_limits_upper_constraint = [&](const std::vector<double>& q) {
            std::vector<double> violation;
            for (size_t i = 0; i < q.size(); ++i) {
                violation.push_back(max_angles[i] - q[i]);
            }
            return violation;
        };

        auto joint_limits_lower_constraint = [&](const std::vector<double>& q) {
            std::vector<double> violation;
            for (size_t i = 0; i < q.size(); ++i) {
                violation.push_back(q[i] - min_angles[i]);
            }
            return violation;
        };

        auto joint_limits_last_orientation = [&](const std::vector<double>& q) {
            double angle_diff = std::abs(std::accumulate(q.begin(), q.end(), 0.0) - end_angle);
            return end_angle_tol - angle_diff;
        };

        std::vector<double> result;
        try {
            result = minimize(distance_to_default, q, y_upper_constraint, y_lower_constraint,
                              joint_limits_upper_constraint, joint_limits_lower_constraint,
                              joint_limits_last_orientation);
        } catch (const std::runtime_error& e) {
            std::cerr << e.what() << std::endl;
        }
        return result;
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

    std::vector<double> minimize(
        std::function<double(const std::vector<double>&)> objective,
        const std::vector<double>& initial_guess,
        std::function<double(const std::vector<double>&)> constraint1,
        std::function<double(const std::vector<double>&)> constraint2,
        std::function<std::vector<double>(const std::vector<double>&)> constraint3,
        std::function<std::vector<double>(const std::vector<double>&)> constraint4,
        std::function<double(const std::vector<double>&)> constraint5
    ) {
        // Implement a suitable optimization algorithm here, e.g., gradient descent, or use a library.
        // This example does not include a specific optimization algorithm.
        // Throw an error to indicate that an optimization library or algorithm should be used.
        throw std::runtime_error("Optimization algorithm not implemented");
    }
};
