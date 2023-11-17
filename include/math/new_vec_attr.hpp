#include <cmath>

void AttractiveForce(float q1, float qd1, float q2, float qd2, double Katt, double a, double b, double c,
                     double linear_slope, double rad_thr, double U_trans, float &grad_q1, float &grad_q2)
{
    double dist_to_goal = sqrt(pow(q1 - qd1, 2) + pow(q2 - qd2, 2));

    double omega = 40;
    double blend = 0.5 * (tanh((dist_to_goal - rad_thr) / omega) + 1);
    double d_sigma_dd = 0.5 * (1 / omega) * pow((1 / cosh((dist_to_goal - rad_thr) / omega)), 2);

    double d_d_dq[2] = {(q1 - qd1) / dist_to_goal, (q2 - qd2) / dist_to_goal};
    double d_sigma_dq[2] = {d_sigma_dd * d_d_dq[0], d_sigma_dd * d_d_dq[1]};

    grad_q1 = -Katt * c * 2 * (q1 - qd1) * (1 - blend) / (a * a) + d_sigma_dq[0] * (linear_slope * (dist_to_goal - rad_thr) * U_trans) + blend * linear_slope * d_d_dq[0] * U_trans;
    grad_q2 = -Katt * c * 2 * (q2 - qd2) * (1 - blend) / (b * b) + d_sigma_dq[1] * (linear_slope * (dist_to_goal - rad_thr) * U_trans) + blend * linear_slope * d_d_dq[1] * U_trans;
}
