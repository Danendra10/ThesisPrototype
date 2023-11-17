#include <iostream>
#include <opencv2/opencv.hpp>

cv::Mat RepulsiveField(double d, double d0, double Krep)
{
    double Urep;
    if (d <= d0)
    {
        Urep = 0.5 * Krep * (1 / d - 1 / d0) * (1 / d - 1 / d0);
    }
    else
    {
        Urep = 0;
    }
    Urep = std::max(0.0, std::min(Urep, 8000.0));
    return cv::Mat(1, 1, CV_64F, Urep);
}

cv::Mat NewAttractiveForce(const cv::Mat &q, const cv::Mat &qd, double Katt, double a, double b, double c,
                           double linear_slope, double rad_thr, double U_trans)
{
    double dist_to_goal = cv::norm(q - qd);

    double omega = 40;
    double blend = 0.5 * (tanh((dist_to_goal - rad_thr) / omega) + 1);
    double d_sigma_dd = 0.5 * (1 / omega) * pow((1 / cosh((dist_to_goal - rad_thr) / omega)), 2);

    cv::Mat d_d_dq = (q - qd) / dist_to_goal;
    cv::Mat d_sigma_dq = d_sigma_dd * d_d_dq;

    cv::Mat grad_Uattr_q = Katt * c * 2 * (q - qd) * (1 - blend) / (a * a) + d_sigma_dq * (linear_slope * (dist_to_goal - rad_thr) * U_trans) + blend * linear_slope * d_d_dq * U_trans;

    return -grad_Uattr_q;
}

int main()
{
    int field_x = 1200;
    int field_y = 800;
    int target_x = field_x / 2;
    int target_y = field_y / 2;

    cv::Mat qd = (cv::Mat_<double>(2, 1) << target_x, target_y);

    // Constants
    double a = 1;
    double b = 1;
    double c = 0.04;
    double k_attr = 1;
    double linear_slope = 1;
    double rad_thr = 50;
    double U_trans = 100;
    double k_repl = 100000000;
    double d0 = 200;

    // Obstacles
    cv::Mat obstacles = (cv::Mat_<double>(5, 2) << 200, 700, 400, 150, 150, 400, 800, 700, 1000, 800);

    cv::Mat f_total_x = cv::Mat::zeros(field_y, field_x, CV_64F);
    cv::Mat f_total_y = cv::Mat::zeros(field_y, field_x, CV_64F);

    for (int i = 0; i < field_x; i += 20)
    {
        for (int j = 0; j < field_y; j += 20)
        {
            cv::Mat q = (cv::Mat_<double>(2, 1) << i, j);
            cv::Mat f_attr = NewAttractiveForce(q, qd, k_attr, a, b, c, linear_slope, rad_thr, U_trans);

            cv::Mat f_repl = cv::Mat::zeros(2, 1, CV_64F);

            for (int k = 0; k < obstacles.rows; ++k)
            {
                cv::Mat obstacle = obstacles.row(k).t();
                double d = cv::norm(q - obstacle);
                cv::Mat rep_field = RepulsiveField(d, d0, k_repl);
                f_repl += rep_field;
            }

            cv::Mat f_total = f_attr - f_repl;
            f_total_x.at<double>(j, i) = f_total.at<double>(0, 0);
            f_total_y.at<double>(j, i) = f_total.at<double>(1, 0);
        }
    }

    cv::Mat fieldImage(field_y, field_x, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int i = 0; i < field_x; i += 20)
    {
        for (int j = 0; j < field_y; j += 20)
        {
            cv::Point start(i, j);
            cv::Point end(i + f_total_x.at<double>(j, i), j + f_total_y.at<double>(j, i));
            cv::arrowedLine(fieldImage, start, end, cv::Scalar(0, 0, 0), 1, 8, 0, 1);
        }
    }

    cv::circle(fieldImage, cv::Point(qd.at<double>(0), qd.at<double>(1)), 5, cv::Scalar(0, 255, 0), -1);

    for (int k = 0; k < obstacles.rows; ++k)
    {
        cv::Point obstacle(obstacles.at<double>(k, 0), obstacles.at<double>(k, 1));
        cv::circle(fieldImage, obstacle, 5, cv::Scalar(0, 0, 255), -1);
    }

    cv::imshow("Field", fieldImage);
    cv::waitKey(0);
    return 0;
}
