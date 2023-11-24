#include <cmath>
#include <advanced_math/advanced_math.hpp>
#include <iostream>

void AttractiveForce(float q1, float qd1, float q2, float qd2,
                     double linear_slope, double rad_thr, double U_trans, float &grad_q1, float &grad_q2, double Katt = 0.3, double a = 1, double b = 1, double c = 0.001)
{
    float x_diff = (q1 - qd1);
    float y_diff = (q2 - qd2);

    double dist_to_goal = __builtin_sqrtf(x_diff * x_diff + y_diff * y_diff);

    double omega = 40;
    double dist_diff = (dist_to_goal - rad_thr);

    int index = static_cast<int>((dist_diff / omega + 26.0) / 0.01); // Adjust index based on range and step

    // double blend = 0.5 * (tanh(dist_diff / omega) + 1);
    double blend = 0.5 * (tanh_lut[index] + 1);
    double d_sigma_dd = 0.5 * (1 / omega) * pow((1 / cosh_lut[index]), 2);

    double d_d_dq[2] = {x_diff / dist_to_goal, y_diff / dist_to_goal};
    double d_sigma_dq[2] = {d_sigma_dd * d_d_dq[0], d_sigma_dd * d_d_dq[1]};

    grad_q1 = -Katt * c * 2 * x_diff * (1 - blend) / (a * a) + d_sigma_dq[0] * (linear_slope * dist_diff * U_trans) + blend * linear_slope * d_d_dq[0] * U_trans;
    grad_q2 = -Katt * c * 2 * y_diff * (1 - blend) / (b * b) + d_sigma_dq[1] * (linear_slope * dist_diff * U_trans) + blend * linear_slope * d_d_dq[1] * U_trans;
}
