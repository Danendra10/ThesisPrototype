#include <simple_example/final/attack_single.hpp>

int main()
{
    logger_instance.Log(logger::YELLOW, "Initializing");
    while (true)
    {
        MainGameCallback();
    }
}

void MainGameCallback()
{
    // ClearFrame();
    DrawField();

    MainAlgorithm();
    DrawOurRobot();
    DrawBall();

    CheckRobotHoldingBall();
    UpdateBallPosWhenHeld();

    Display();
}

void MainAlgorithm()
{
    float target_x = 0;
    float target_y = 0;
    CountRobotTarget(target_x, target_y);
    robot_target.x = target_x;
    robot_target.y = target_y;
    CountDesiredVel();
}

void CountDesiredVel()
{
    float f_x_attr, f_y_attr;
    AttractiveForce(movement_points.x, robot_target.x, movement_points.y, robot_target.y, Kattr, a, b, c, linear_slope, rad_thr, u_trans, f_x_attr, f_y_attr);
    // normalize fx and fy from -200 to 200 to -5 to 5
    f_x_attr = (f_x_attr / 200) * 5;
    f_y_attr = (f_y_attr / 200) * 5;
    float f_x_rep_total = 0, f_y_rep_total = 0;
    for (const auto &obstacle : obstacles)
    {
        float f_x_rep, f_y_rep;
        RepulsiveForce(movement_points.x, obstacle.x, movement_points.y, obstacle.y, Krepl, d0, f_x_rep, f_y_rep);
        f_x_rep_total += f_x_rep * -1;
        f_y_rep_total += f_y_rep * -1;
    }

    float f_x_total = -f_x_attr + f_x_rep_total;
    float f_y_total = -f_y_attr + f_y_rep_total;

    robot_vel.x = f_x_total;
    robot_vel.y = f_y_total;
    movement_points.x += robot_vel.x;
    movement_points.y += robot_vel.y;
    vec_movement_points.push_back(movement_points);

    for (int i = 0; i < vec_movement_points.size(); i++)
    {
        circle(main_frame, Point(vec_movement_points[i].x, vec_movement_points[i].y), 5, CV_BLACK, -1, 8, 0);
    }

    printf("Vel X: %f, Vel Y: %f\n", robot_vel.x, robot_vel.y);
}

void CountRobotTarget(float &target_x, float &target_y)
{
    uint8_t robot_able_to_shoot = 0;

    if (robot[0].pose_x >= X_FIELD_3_4)
        robot_able_to_shoot = 1;

    if (robot_able_to_shoot)
        return;

    float least_force_x = __FLT_MAX__;
    float least_force_y = __FLT_MAX__;
    float least_force_x_idx = __FLT_MAX__;
    float least_force_y_idx = __FLT_MAX__;

    Point2f midpoint = MidPointTriangle(goal_pose.x, goal_pose.y, X_FIELD_1_2, Y_FIELD - 50, X_FIELD_1_2, 50);

    line(main_frame, Point(X_FIELD_1_2, Y_FIELD - 50), Point(X_FIELD_1_2, 50), CV_BLACK, 2, 8, 0);
    line(main_frame, Point(goal_pose.x, goal_pose.y), Point(X_FIELD_1_2, Y_FIELD - 50), CV_BLACK, 2, 8, 0);
    line(main_frame, Point(goal_pose.x, goal_pose.y), Point(X_FIELD_1_2, 50), CV_BLACK, 2, 8, 0);

    float x_target = (!robot[0].is_holding_ball ? ball.pose_x : midpoint.x);
    float y_target = (!robot[0].is_holding_ball ? ball.pose_y : midpoint.y);

    circle(main_frame, Point(x_target, y_target), 10, CV_BLACK, -1, 8, 0);

    for (int i = 0; i <= X_FIELD; i += stepSize)
    {
        for (int j = 0; j <= Y_FIELD; j += stepSize)
        {
            // Current position
            float x_curr = i;
            float y_curr = j;

            if (x_curr >= X_FIELD - 300)
                continue;

            // if (IsOutsideRobotMaxRadius(x_curr, y_curr, robot[0].pose_x, robot[0].pose_y, max_robot_movement_radius))
            //     continue;

            if (robot[0].pose_x <= X_FIELD_3_4 && x_curr <= robot[0].pose_x)
                continue;
            if (robot[0].pose_x <= X_FIELD_1_4 && x_curr <= X_FIELD_1_4)
                continue;

            // Calculate attractive force
            float f_x_attr, f_y_attr;
            AttractiveForce(x_curr, x_target, y_curr, y_target, Kattr, a, b, c, linear_slope, rad_thr, u_trans, f_x_attr, f_y_attr);

            f_attr_x.at<float>(i, j) = f_x_attr;
            f_attr_y.at<float>(i, j) = f_y_attr;

            // Calculate repulsive force from each obstacle
            float f_x_rep_total = 0, f_y_rep_total = 0;

            // logger_instance.Log(logger::YELLOW, "X %f Y %f IsOutsideRobotMaxRadius: %f", x_curr, y_curr, sqrt(pow(x_curr - robot_pose.x, 2) + pow(y_curr - robot_pose.y, 2)));

            uint8_t is_in_obstacle_zone = 0;
            for (const auto &obstacle : obstacles)
            {
                // when the current
                // if (sqrt((x_curr - obstacle.x) * (x_curr - obstacle.x) + (y_curr - obstacle.y) * (y_curr - obstacle.y)) < exclude_radius_around_obstacles ||
                //     CheckLineCircleIntersection2(x_curr, y_curr, robot[0].pose_x, robot[0].pose_y, obstacle.x, obstacle.y, exclude_radius_around_obstacles))
                // {
                //     is_in_obstacle_zone = 1;
                //     break;
                // }

                float f_x_rep, f_y_rep;
                // repulsive_force.Update(x_curr, y_curr, obstacle.x, obstacle.y, f_x_rep, f_y_rep);
                RepulsiveForce(x_curr, obstacle.x, y_curr, obstacle.y, Krepl, d0, f_x_rep, f_y_rep);
                f_x_rep_total += f_x_rep * -1;
                f_y_rep_total += f_y_rep * -1;
            }

            if (is_in_obstacle_zone)
                continue;

            f_rep_x.at<float>(i, j) = f_x_rep_total;
            f_rep_y.at<float>(i, j) = f_y_rep_total;

            // Total force
            float f_x_total = f_attr_x.at<float>(i, j) + f_rep_x.at<float>(i, j);
            float f_y_total = f_attr_y.at<float>(i, j) + f_rep_y.at<float>(i, j);

            f_total_x.at<float>(i, j) = f_x_total;
            f_total_y.at<float>(i, j) = f_y_total;

            if (abs(f_x_total) <= abs(least_force_x) && abs(f_y_total) <= abs(least_force_y) && sqrt(pow(x_curr - robot[0].pose_x, 2) + pow(y_curr - robot[0].pose_y, 2)) > max_robot_movement_radius - 100)
            {
                least_force_x = f_x_total;
                least_force_x_idx = i;
                least_force_y = f_y_total;
                least_force_y_idx = j;
            }
            // logger_instance.Log(logger::RED, "Least: %f %f", f_x_total, f_y_total);
            line(main_frame, Point(i, j), Point(i + f_x_total, j + f_y_total), Scalar(0, 0, 255), 1);
        }
    }

    // circle(main_frame, Point2d(least_force_x_idx, least_force_y_idx), 10, CV_MAGENTA, -1);
    // line(main_frame, Point2d(robot[0].pose_x, robot[0].pose_y), Point2d(least_force_x_idx, least_force_y_idx), CV_MAGENTA, 5);

    target_x = least_force_x_idx;
    target_y = least_force_y_idx;
}

void ClearFrame()
{
    main_frame = Scalar(0);
}

void DrawField()
{
    const int penalty_area_width = 150;  // Assuming 100 pixels = 1 meter scale, adjust if needed
    const int penalty_area_height = 350; // Based on your field diagram

    const int goal_area_width = 50;   // As per field diagram
    const int goal_area_height = 180; // 1.8 meters

    const int penalty_spot_distance = 260; // From the goal line
    const int corner_arc_radius = 50;      // 0.5 meters

    // Calculate positions based on the field size
    int penalty_area_top_y = (Y_FIELD / 2) - (penalty_area_height / 2);
    int goal_area_top_y = (Y_FIELD / 2) - (goal_area_height / 2);

    rectangle(main_frame, Rect(Point2i(0, 0), Point2i(X_FIELD, Y_FIELD)), CV_GREEN, -1);
    // create outer line with distance of 50 from the outer box
    rectangle(main_frame, Rect(Point2i(50, 50), Point2i(X_FIELD - 50, Y_FIELD - 50)), CV_WHITE, 5);
    // center line
    line(main_frame, Point(X_FIELD_1_2, 50), Point(X_FIELD_1_2, Y_FIELD - 50), CV_WHITE, 5);
    // center circle
    circle(main_frame, Point(X_FIELD_1_2, Y_FIELD_1_2), 260, CV_WHITE, 5);
    circle(main_frame, Point(X_FIELD_1_2, Y_FIELD_1_2), 10, CV_WHITE, -1);
    // Draw penalty and goal areas for both sides
    rectangle(main_frame, Point(50, penalty_area_top_y), Point(50 + penalty_area_width, penalty_area_top_y + penalty_area_height), CV_WHITE, 5);
    rectangle(main_frame, Point(X_FIELD - 50 - penalty_area_width, penalty_area_top_y), Point(X_FIELD - 50, penalty_area_top_y + penalty_area_height), CV_WHITE, 5);

    rectangle(main_frame, Point(50, goal_area_top_y), Point(50 + goal_area_width, goal_area_top_y + goal_area_height), CV_WHITE, 5);
    rectangle(main_frame, Point(X_FIELD - 50 - goal_area_width, goal_area_top_y), Point(X_FIELD - 50, goal_area_top_y + goal_area_height), CV_WHITE, 5);

    // Penalty spots
    circle(main_frame, Point(50 + penalty_spot_distance, Y_FIELD / 2), 5, CV_WHITE, -1);
    circle(main_frame, Point(X_FIELD - 50 - penalty_spot_distance, Y_FIELD / 2), 5, CV_WHITE, -1);

    // Corner arcs
    ellipse(main_frame, Point(50, 50), Size(corner_arc_radius, corner_arc_radius), 0, 0, 90, CV_WHITE, 5);
    ellipse(main_frame, Point(50, Y_FIELD - 50), Size(corner_arc_radius, corner_arc_radius), 180, 90, 180, CV_WHITE, 5);
    ellipse(main_frame, Point(X_FIELD - 50, 50), Size(corner_arc_radius, corner_arc_radius), 90, 90, 0, CV_WHITE, 5);
    ellipse(main_frame, Point(X_FIELD - 50, Y_FIELD - 50), Size(corner_arc_radius, corner_arc_radius), 0, 180, 270, CV_WHITE, 5);
}

void DrawOurRobot()
{
    DrawRobot(robot[0].pose_x, robot[0].pose_y, robot[0].pose_th, CV_MAGENTA);
    // circle(main_frame, Point(robot[0].pose_x, robot[0].pose_y), 20, CV_MAGENTA, -1);
    // circle(main_frame, Point(robot[1].pose_x, robot[1].pose_y), 20, CV_MAGENTA, -1);

    circle(main_frame, Point(enemy[0].pose_x, enemy[0].pose_y), 20, Scalar(0, 0, 255), -1); // Draw obstacles
    putText(main_frame, "x = " + std::to_string(enemy[0].pose_x) + " y = " + std::to_string(enemy[0].pose_y), Point(enemy[0].pose_x, enemy[0].pose_y), 1, 1, CV_WHITE);
    circle(main_frame, Point(enemy[1].pose_x, enemy[1].pose_y), 20, Scalar(0, 0, 255), -1); // Draw obstacles
    putText(main_frame, "x = " + std::to_string(enemy[1].pose_x) + " y = " + std::to_string(enemy[1].pose_y), Point(enemy[1].pose_x, enemy[1].pose_y), 1, 1, CV_WHITE);
}

void DrawBall()
{
    float pose_x = ball.pose_x;
    float pose_y = ball.pose_y;
    if (num_robot_holding_ball != -1)
    {
        pose_x = robot[num_robot_holding_ball].pose_x + 5;
        pose_y = robot[num_robot_holding_ball].pose_y + 5;
    }
    circle(main_frame, Point(pose_x, pose_y), 10, CV_YELLOW, -1);
}

void Display()
{
    imshow("Main", main_frame);
    if (waitKey(1) == 'q')
    {
        exit(0);
    }
}

void DrawRobot(float x_, float y_, float th_, Scalar color_)
{
    circle(main_frame, Point2f((x_), (y_)), 30, color_, 4);
    line(main_frame, Point2f((x_), (y_)), Point((x_ + 35 * cos(th_ * DEG2RAD)), (y_ + 35 * sin(th_ * DEG2RAD))), color_, 3);
}