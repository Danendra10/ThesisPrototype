#include <simple_example/final/defend_single.hpp>

int main()
{
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

    enemy[0].pose_x = x_enemy_1;
    enemy[0].pose_y = y_enemy_1;
    enemy[1].pose_x = x_enemy_2;
    enemy[1].pose_y = y_enemy_2;

    robot[0].pose_x = X_FIELD_1_4;
    robot[0].pose_y = Y_FIELD_1_2;

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
        circle(main_frame, Point(attacker_vec_movement_points[i].x, attacker_vec_movement_points[i].y), 5, CV_BLACK, -1, 8, 0);
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

void MainAlgorithm()
{
    if (robot[0].is_holding_ball)
    {
        robot_target_x = X_FIELD;
        robot_target_y = Y_FIELD_1_2;
    }
    else
    {
        robot_target_x = ball.pose_x;
        robot_target_y = ball.pose_y;
    }
    float f_attr_x;
    float f_attr_y;
    static PID_t angles_pid;
    PIDInit(&angles_pid, 3.2, 0, 0);
    float angle_error = atan2(robot_target_y - robot[0].pose_y, robot_target_x - robot[0].pose_x) * RAD2DEG - robot[0].pose_th;
    AttractiveForce(robot[0].pose_x, robot_target_x, robot[0].pose_y, robot_target_y, 2, 50, 100, f_attr_x, f_attr_y);
    f_attr_x = (f_attr_x / 200) * 5;
    f_attr_y = (f_attr_y / 200) * 5;

    float f_x_rep_total = 0, f_y_rep_total = 0;

    for (size_t i = 0; i < sizeof(enemy) / sizeof(enemy[0]); i++)
    {
        if (sqrt(pow(enemy[i].pose_x - ball.pose_x, 2) + pow(enemy[i].pose_y - ball.pose_y, 2)) < 50)
            continue;
        float f_rep_x;
        float f_rep_y;
        RepulsiveForce(robot[0].pose_x, enemy[i].pose_x, robot[0].pose_y, enemy[i].pose_y, 100000000, 400, f_rep_x, f_rep_y);
        f_x_rep_total += f_rep_x * -1;
        f_y_rep_total += f_rep_y * -1;
    }

    float f_x_total = -f_attr_x + f_x_rep_total;
    float f_y_total = -f_attr_y + f_y_rep_total;
    float vel_th = PIDCalculate(&angles_pid, angle_error, 10);

    robot[0].pose_x += f_x_total;
    robot[0].pose_y += f_y_total;
    robot[0].pose_th += vel_th;

    logger_instance.Log(logger::RED, "Robot Position: %f %f || target: %d %d", robot[0].pose_x, robot[0].pose_y, robot_target_x, robot_target_y);

    attacker_movement_points.x = robot[0].pose_x;
    attacker_movement_points.y = robot[0].pose_y;

    attacker_vec_movement_points.push_back(attacker_movement_points);
}