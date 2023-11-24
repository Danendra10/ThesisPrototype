#include <cmath>

void RepulsiveForce(float q1, float qd1, float q2, float qd2, float k_rep,
                    float d_thresh, float &grad_q1, float &grad_q2)
{
    // Vector from obstacle to current position
    float vec_x = q1 - qd1;
    float vec_y = q2 - qd2;

    float d = sqrt(vec_x * vec_x + vec_y * vec_y);

    float repulsive_force = k_rep * (1 / d - 1 / d_thresh) * (1 / (d * d));

    grad_q1 = -(repulsive_force * (vec_x / d)) * (d <= d_thresh && d != 0);
    grad_q2 = -(repulsive_force * (vec_y / d)) * (d <= d_thresh && d != 0);
}