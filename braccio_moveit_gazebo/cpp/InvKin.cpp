#include <iostream>
#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>

class Arm3Link {
public:
    Arm3Link(std::vector<double> L = {1.0, 1.0, 0.8}) : q{0.0, 0.0, 0.0}, L(L),
                                                        max_y(1.0), min_y(0.0),
                                                        end_angle_tol(0.05),
                                                        end_angle(-M_PI / 2),
                                                        max_angles({1.6, M_PI / 2, M_PI / 2}),
                                                        min_angles({0.27, -M_PI / 2, -M_PI / 2}) {
    }

    std::vector<double> get_xy(std::vector<double> q = {}) {
        if (q.empty())
            q = this->q;

        double x = L[0] * cos(q[0]) +
                   L[1] * cos(q[0] + q[1]) +
                   L[2] * cos(std::accumulate(q.begin(), q.end(), 0.0));

        double y = L[0] * sin(q[0]) +
                   L[1] * sin(q[0] + q[1]) +
                   L[2] * sin(std::accumulate(q.begin(), q.end(), 0.0));

        return {x, y};
    }

    std::vector<double> inv_kin(double x, double min_y, double max_y, double end_angle) {
        auto distance_to_default = [&](const std::vector<double>& q) {
            double x_calc = L[0] * cos(q[0]) +
                            L[1] * cos(q[0] + q[1]) +
                            L[2] * cos(std::accumulate(q.begin(), q.end(), 0.0));
            return pow(x_calc - x, 2);
        };

        auto y_upper_constraint = [&](const std::vector<double>& q) {
            double y_calc = L[0] * sin(q[0]) +
                            L[1] * sin(q[0] + q[1]) +
                            L[2] * sin(std::accumulate(q.begin(), q.end(), 0.0));
            return max_y - y_calc;
        };

        auto y_lower_constraint = [&](const std::vector<double>& q) {
            double y_calc = L[0] * sin(q[0]) +
                            L[1] * sin(q[0] + q[1]) +
                            L[2] * sin(std::accumulate(q.begin(), q.end(), 0.0));
            return y_calc - min_y;
        };

        auto joint_limits_upper_constraint = [&](const std::vector<double>& q) {
            std::vector<double> constraints;
            for (size_t i = 0; i < q.size(); ++i)
                constraints.push_back(max_angles[i] - q[i]);
            return constraints;
        };

        auto joint_limits_lower_constraint = [&](const std::vector<double>& q) {
            std::vector<double> constraints;
            for (size_t i = 0; i < q.size(); ++i)
                constraints.push_back(q[i] - min_angles[i]);
            return constraints;
        };

        auto joint_limits_last_orientation = [&](const std::vector<double>& q) {
            return end_angle_tol - std::abs(std::accumulate(q.begin(), q.end(), 0.0) - end_angle);
        };

        this->min_y = min_y;
        this->max_y = max_y;
        if (end_angle != -1)
            this->end_angle = end_angle;

        std::vector<double> q_optimized = q;
        // Optimization using a suitable solver
        // Replace this with your preferred optimization algorithm
        // Here, I'm using a basic gradient-based solver as an example
        std::vector<double> q_new(q_optimized.size(), 0.0);
        double learning_rate = 0.01;
        double epsilon = 0.001;

        do {
            for (size_t i = 0; i < q_optimized.size(); ++i) {
                double gradient = (distance_to_default(q_optimized) - distance_to_default(q_new)) / epsilon;
                q_new[i] = q_optimized[i] - learning_rate * gradient;

                // Apply constraints
                q_new[i] = std::max(min_angles[i], std::min(max_angles[i], q_new[i]));
            }

            q_optimized = q_new;
        } while (distance_to_default(q_optimized) > epsilon);

        this->q = q_optimized;
        return this->q;
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
};

int main() {
    Arm3Link arm;

    std::vector<double> q = arm.inv_kin(2.0, 0.0, 1.5, -M_PI / 4);
    std::vector<double> xy = arm.get_xy(q);

    std::cout << "Joint angles: ";
    for (const auto& angle : q)
        std::cout << angle << " ";
    std::cout << std::endl;

    std::cout << "End effector position: (" << xy[0] << ", " << xy[1] << ")" << std::endl;

    return 0;
}