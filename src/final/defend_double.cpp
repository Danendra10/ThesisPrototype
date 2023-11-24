#include <simple_example/final/defend_double.hpp>

int main()
{
    namedWindow("Slider");
    createTrackbar("Y target", "Slider", &robot_target_y, Y_FIELD, OnTrackbarCallback);
    createTrackbar("Variasi", "Slider", &index_variasi, 4, OnTrackbarCallback);
    namedWindow("Data Frame");
    while (true)
    {
        MainGameCallback();

        auto end = steady_clock::now();

        // Cast the duration to floating-point milliseconds
        duration<float, milli> time_diff_ms = end - start;

        // write the time difference to the DataFrame
        Mat frame = Mat(500, 500, CV_8UC3, Scalar(255, 255, 255));
        putText(frame, "Time taken: " + to_string(time_diff_ms.count()), Point(10, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2, LINE_AA);

        // printf("portential vec size: %lu\n", potential_force_vect.size());
        imshow("Data Frame", frame);

        // Use %f to print floating-point values
        logger_instance.Log(logger::YELLOW, "Time taken: %.3f ms", time_diff_ms.count());
    }
}

void OnTrackbarCallback(int, void *)
{
    start = steady_clock::now();
    trackbarChanged = true;

    enemy[0].pose_x = variasi_halangan_1_x[index_variasi];
    enemy[0].pose_y = variasi_halangan_1_y[index_variasi];
    enemy[1].pose_x = variasi_halangan_2_x[index_variasi];
    enemy[1].pose_y = variasi_halangan_2_y[index_variasi];

    robot[0].pose_x = 50;
    robot[0].pose_y = 50;
    robot[0].pose_th = 0;
    robot[1].pose_x = 50;
    robot[1].pose_y = 850;
    robot[1].pose_th = 0;

    assist_vec_movement_points.clear();
    attacker_vec_movement_points.clear();

    attacker_movement_points.x = robot[0].pose_x;
    attacker_movement_points.y = robot[0].pose_y;

    assist_movement_points.x = robot[1].pose_x;
    assist_movement_points.y = robot[1].pose_y;
}

void MainGameCallback()
{
    DrawField();

    MainAlgorithm();
    DrawOurRobot();
    DrawBall();
    DrawTarget();

    CheckRobotHoldingBall();
    UpdateBallPosWhenHeld();

    for (size_t i = 0; i < attacker_vec_movement_points.size(); i++)
    {
        circle(main_frame, attacker_vec_movement_points[i], 5, CV_BLACK, -1, 8, 0);
    }
    for (size_t i = 0; i < assist_vec_movement_points.size(); i++)
    {
        circle(main_frame, assist_vec_movement_points[i], 5, CV_CYAN, -1, 8, 0);
    }
    Display();

    if (robot[0].is_holding_ball)
        waitKey(0);
    else
    {
        if (waitKey(1) == 'q')
        {
            exit(0);
        }
    }
}

void CountTarget()
{
    attacker_robot_target.x = ball.pose_x;
    attacker_robot_target.y = ball.pose_y;

    assist_robot_target.x = (ball.pose_x + -50) / 2;
    assist_robot_target.y = (ball.pose_y + Y_FIELD_1_2) / 2;
}

float RobotAngleToBall(int num)
{
    return atan2(ball.pose_y - robot[num].pose_y, ball.pose_x - robot[num].pose_x) * RAD2DEG;
}

void MoveRobot(int num)
{
    float target_x = (num ? assist_robot_target.x : attacker_robot_target.x);
    float target_y = (num ? assist_robot_target.y : attacker_robot_target.y);
    float target_th = (num ? RobotAngleToBall(num) : atan2(target_y - robot[num].pose_y, target_x - robot[num].pose_x) * RAD2DEG);
    float f_attr_x;
    float f_attr_y;
    static PID_t angles_pid;
    PIDInit(&angles_pid, 3.2, 0, 0);
    float angle_error = target_th - robot[num].pose_th;
    AttractiveForce(robot[num].pose_x, target_x, robot[num].pose_y, target_y, 2, 50, 100, f_attr_x, f_attr_y);
    f_attr_x = (f_attr_x / 200) * 5;
    f_attr_y = (f_attr_y / 200) * 5;

    float f_x_rep_total = 0, f_y_rep_total = 0;

    for (size_t i = 0; i < sizeof(enemy) / sizeof(enemy[0]); i++)
    {
        if (sqrt(pow(enemy[i].pose_x - ball.pose_x, 2) + pow(enemy[i].pose_y - ball.pose_y, 2)) < 50)
            continue;
        float f_rep_x;
        float f_rep_y;
        RepulsiveForce(robot[num].pose_x, enemy[i].pose_x, robot[num].pose_y, enemy[i].pose_y, 100000000, 400, f_rep_x, f_rep_y);
        f_x_rep_total += f_rep_x * -1;
        f_y_rep_total += f_rep_y * -1;
    }

    if (num)
    {
        float f_rep_x;
        float f_rep_y;
        RepulsiveForce(robot[num].pose_x, robot[0].pose_x, robot[num].pose_y, robot[0].pose_y, 100000000, 400, f_rep_x, f_rep_y);
        f_x_rep_total += f_rep_x * -1;
        f_y_rep_total += f_rep_y * -1;
        RepulsiveForce(robot[num].pose_x, 50, robot[num].pose_y, Y_FIELD_1_2, 10000000, 400, f_rep_x, f_rep_y);
        f_x_rep_total += f_rep_x * -1;
        f_y_rep_total += f_rep_y * -1;
    }

    float f_x_total = -f_attr_x + f_x_rep_total;
    float f_y_total = -f_attr_y + f_y_rep_total;
    float vel_th = PIDCalculate(&angles_pid, angle_error, 1);

    robot[num].pose_x += f_x_total;
    robot[num].pose_y += f_y_total;
    robot[num].pose_th += vel_th;

    logger_instance.Log(logger::RED, "Robot Position[%d]: %f %f || target: %f %f || obs %f %f %f %f", num, robot[num].pose_x, robot[num].pose_y, target_x, target_y, enemy[0].pose_x, enemy[0].pose_y, enemy[1].pose_x, enemy[1].pose_y);

    if (!num)
    {
        attacker_movement_points.x = robot[0].pose_x;
        attacker_movement_points.y = robot[0].pose_y;
        attacker_vec_movement_points.push_back(attacker_movement_points);
    }
    else
    {
        assist_movement_points.x = robot[1].pose_x;
        assist_movement_points.y = robot[1].pose_y;
        assist_vec_movement_points.push_back(assist_movement_points);
    }
}

void MainAlgorithm()
{
    CountTarget();

    MoveRobot(0);
    MoveRobot(1);
}