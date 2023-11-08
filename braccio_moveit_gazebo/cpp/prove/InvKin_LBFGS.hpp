#include <iostream>
#include <vector>
#include <cmath>
#include <lbfgs.h>
// #include <yixuan-LBFGSpp-69e2770/include/LBFGS.h>

class Arm3Link {
public:
    Arm3Link(const std::vector<double>& L) : L(L) {
        q = {0, 0, 0};
        max_y = 1.0;
        min_y = 0.0;
        end_angle_tol = 0.05;
        end_angle = -M_PI / 2.0;
        max_angles = {1.6, M_PI / 2.0, M_PI / 2.0};
        min_angles = {0.27, -M_PI / 2.0, -M_PI / 2.0};
    }

    std::vector<double> get_xy(const std::vector<double>& q) {
        double x = L[0] * cos(q[0]) + L[1] * cos(q[0] + q[1]) + L[2] * cos(q[0] + q[1] + q[2]);
        double y = L[0] * sin(q[0]) + L[1] * sin(q[0] + q[1]) + L[2] * sin(q[0] + q[1] + q[2]);
        return {x, y};
    }

    static lbfgsfloatval_t evaluate(
        void *instance,
        const lbfgsfloatval_t *q,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
    ) {
        Arm3Link *arm = (Arm3Link *)instance;
        lbfgsfloatval_t fx = arm->distance_to_default(q);
        return fx;
    }

    lbfgsfloatval_t distance_to_default(const lbfgsfloatval_t *q) {
        std::vector<double> x_current = get_xy({q[0], q[1], q[2]});
        lbfgsfloatval_t dist = (x_current[0] - x_target) * (x_current[0] - x_target);
        return dist;
    }

    std::vector<double> optimizeIK(double x, double min_y, double max_y, double end_angle) {
        x_target = x;
        min_y = min_y;
        max_y = max_y;
        end_angle = end_angle;

        lbfgs_parameter_t param;
        lbfgs_parameter_init(&param);
        param.max_iterations = 100;

        lbfgsfloatval_t *q_values = new lbfgsfloatval_t[q.size()];

        for (size_t i = 0; i < q.size(); i++) {
            q_values[i] = q[i];
        }

        lbfgsfloatval_t fx;
        lbfgs(param, q_values, &fx, evaluate, this, NULL);

        for (size_t i = 0; i < q.size(); i++) {
            q[i] = q_values[i];
        }

        delete[] q_values;

        return q;
    }

private:
    std::vector<double> q;
    std::vector<double> L;
    lbfgsfloatval_t max_y;
    lbfgsfloatval_t min_y;
    lbfgsfloatval_t end_angle_tol;
    lbfgsfloatval_t end_angle;
    std::vector<lbfgsfloatval_t> max_angles;
    std::vector<lbfgsfloatval_t> min_angles;
    lbfgsfloatval_t x_target;
};

// int main() {
//     std::vector<double> L = {1.0, 1.0, 0.8};
//     Arm3Link arm(L);

//     double x = 2.0;
//     double min_y = 0.0;
//     double max_y = 3.0;
//     double end_angle = -M_PI / 2.0;

//     std::vector<double> q = arm.optimizeIK(x, min_y, max_y, end_angle);

//     std::cout << "Optimized joint angles: ";
//     for (double angle : q) {
//         std::cout << angle << " ";
//     }
//     std::cout << std::endl;

//     return 0;
// }