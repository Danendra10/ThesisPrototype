#include <simple_example/positioning.hpp>

int main()
{
    namedWindow("Parameters", WINDOW_AUTOSIZE);
    createTrackbar("Step Size", "Parameters", &stepSize, 100, OnUpdate);
    createTrackbar("Max Velocity", "Parameters", &maxVel, 200, OnUpdate);
    createTrackbar("Kattr", "Parameters", &Kattr, 1000, OnUpdate);
    createTrackbar("Krepl", "Parameters", &Krepl, 100000000, OnUpdate);
    createTrackbar("Repulsive Radius", "Parameters", &d0, 400, OnUpdate);
    createTrackbar("a", "Parameters", &a, 10, OnUpdate);
    createTrackbar("b", "Parameters", &b, 10, OnUpdate);
    createTrackbar("Ball speed", "Parameters", &_speed, 100, OnUpdate);

    attractive_force.Init(Kattr, a, b, 300);
    repulsive_force.Init(Krepl, d0);

    while (true)
    {
        // visualizeVectorField(f_attr_x, f_attr_y, obstacles, goal);
        int key = cv::waitKey(1);
        if (key == 'q')
        {
            break;
        }
        else if (key == 'w') // Move up
        {
            robot_pose.y = fmaxf(0, robot_pose.y - _speed);
        }
        else if (key == 's') // Move down
        {
            robot_pose.y = fminf(Y_FIELD, robot_pose.y + _speed);
        }
        else if (key == 'a') // Move left
        {
            robot_pose.x = fmaxf(0, robot_pose.x - _speed);
        }
        else if (key == 'd') // Move right
        {
            robot_pose.x = fminf(X_FIELD, robot_pose.x + _speed);
        }

        auto start = std::chrono::high_resolution_clock::now();
        // After moving the ball, redraw the scene with updated ball position
        main_frame = Scalar::all(0); // Clear the frame or set to your background

        float least_force_x = __FLT_MAX__;
        float least_force_y = __FLT_MAX__;
        float least_force_x_idx = __FLT_MAX__;
        float least_force_y_idx = __FLT_MAX__;

        uint16_t total_possible_points = 0;
        uint8_t robot_able_to_shoot = 0;

        for (const auto &obstacle : obstacles)
        {
            circle(main_frame, obstacle, 10, Scalar(0, 0, 255), -1); // Draw obstacles
        }
        circle(main_frame, ball, 10, CV_BLUE, -1); // Draw ball with updated position

        if (robot_pose.x >= X_FIELD_3_4)
            robot_able_to_shoot = 1;

        if (robot_able_to_shoot)
            continue;
        for (int i = 0; i <= X_FIELD; i += stepSize)
        {
            for (int j = 0; j <= Y_FIELD; j += stepSize)
            {
                total_possible_points++;

                // Current position
                float x_curr = i;
                float y_curr = j;

                if (IsOutsideRobotMaxRadius(x_curr, y_curr))
                    continue;

                if (robot_pose.x <= X_FIELD_1_2 && x_curr <= robot_pose.x)
                    continue;
                if (robot_pose.x <= X_FIELD_1_4 && x_curr <= X_FIELD_1_4)
                    continue;

                // Calculate attractive force
                float f_x_attr, f_y_attr;
                attractive_force.Update(x_curr, y_curr, goal_pose.x, goal_pose.y, maxVel, f_x_attr, f_y_attr);

                f_attr_x.at<float>(i, j) = f_x_attr;
                f_attr_y.at<float>(i, j) = f_y_attr;

                // Calculate repulsive force from each obstacle
                float f_x_rep_total = 0, f_y_rep_total = 0;

                // logger_instance.Log(logger::YELLOW, "X %f Y %f IsOutsideRobotMaxRadius: %f", x_curr, y_curr, sqrt(pow(x_curr - robot_pose.x, 2) + pow(y_curr - robot_pose.y, 2)));

                uint8_t is_in_obstacle_zone = 0;
                for (const auto &obstacle : obstacles)
                {
                    // when the current
                    if (sqrt((x_curr - obstacle.x) * (x_curr - obstacle.x) + (y_curr - obstacle.y) * (y_curr - obstacle.y)) < exclude_radius_around_obstacles ||
                        CheckLineCircleIntersection2(x_curr, y_curr, robot_pose.x, robot_pose.y, obstacle.x, obstacle.y, exclude_radius_around_obstacles))
                    {
                        is_in_obstacle_zone = 1;
                        break;
                    }

                    float f_x_rep, f_y_rep;
                    repulsive_force.Update(x_curr, y_curr, obstacle.x, obstacle.y, f_x_rep, f_y_rep);
                    f_x_rep_total += f_x_rep;
                    f_y_rep_total += f_y_rep;
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

                if (abs(f_x_total) <= abs(least_force_x) && abs(f_y_total) <= abs(least_force_y) && sqrt(pow(x_curr - robot_pose.x, 2) + pow(y_curr - robot_pose.y, 2)) > max_robot_movement_radius - 100)
                {
                    least_force_x = f_x_total;
                    least_force_x_idx = i;
                    least_force_y = f_y_total;
                    least_force_y_idx = j;
                }

                line(main_frame, Point(i, j), Point(i + f_x_total, j + f_y_total), Scalar(0, 255, 0), 1);
            }
        }

        auto end = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> duration = end - start;

        // logger_instance.Log(logger::BLUE, "Step Size %d Duration: %s Total possible points %d", stepSize, std::to_string(duration.count()).c_str(), total_possible_points);

        circle(main_frame, Point2d(least_force_x_idx, least_force_y_idx), 10, CV_WHITE, -1);
        line(main_frame, Point2d(robot_pose.x, robot_pose.y), Point2d(least_force_x_idx, least_force_y_idx), CV_WHITE, 1);

        // text on top
        putText(main_frame, "Current position: " + to_string(robot_pose.x) + ", " + to_string(robot_pose.y), Point(200, 20), FONT_HERSHEY_SIMPLEX, 1, CV_RED, 1);
        putText(main_frame, "Friend position: " + to_string(friend_pose.x) + ", " + to_string(friend_pose.y), Point(200, 60), FONT_HERSHEY_SIMPLEX, 1, CV_RED, 1);
        putText(main_frame, "Target point: " + to_string(least_force_x_idx) + ", " + to_string(least_force_y_idx), Point(200, 100), FONT_HERSHEY_SIMPLEX, 1, CV_RED, 1);
        // if (!least_force_x_idx && !least_force_y_idx)
        //     putText(main_frame, "CAN`T PAST", Point(500, 20), FONT_HERSHEY_SIMPLEX, 0.5, CV_RED, 1);
        // else
        //     putText(main_frame, "ABLE TO PASS", Point(500, 40), FONT_HERSHEY_SIMPLEX, 0.5, CV_WHITE, 1);

        Display();
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    destroyAllWindows();
    return 0;
}

void Display()
{
    DrawTriangle(main_frame, robot_pose.x, robot_pose.y, 10, CV_MAGENTA);
    DrawTriangle(main_frame, friend_pose.x, friend_pose.y, 10, CV_MAGENTA);
    imshow("Display", main_frame);
}

// Visualization function
void visualizeVectorField(const Mat &field_x, const Mat &field_y, const vector<Point2f> &obstacles, const Point2f &goal)
{
    // Create an image to draw the vector field
    Mat display = Mat::zeros(field_x.size(), CV_8UC3);

    // Normalize the magnitude of vectors for consistent arrow length
    double maxVal;
    minMaxLoc(field_x, nullptr, &maxVal);
    double scaleFactor = 50.0 / maxVal; // Adjust 50.0 to scale the arrows as needed

    // Draw the vector field
    for (int i = 0; i < field_x.rows; i++)
    {
        for (int j = 0; j < field_x.cols; j++)
        {
            Point2f start(j, i);
            // Scale the force to a visible size
            Point2f end = start + Point2f(field_x.at<float>(i, j), field_y.at<float>(i, j)) * scaleFactor;
            // Draw the line for the vector
            arrowedLine(display, start, end, Scalar(255, 0, 0), 1, LINE_AA);
        }
    }

    // Draw obstacles as circles
    for (const auto &obstacle : obstacles)
    {
        circle(display, obstacle, 10, Scalar(0, 0, 255), -1); // -1 for filled circle
    }

    // Draw the goal as a green circle
    circle(display, goal, 10, Scalar(0, 255, 0), -1);

    // Show the image
    imshow("Vector Field", display);
}

void OnUpdate(int, void *)
{
    // clear the frame
    main_frame = Mat(Y_FIELD, X_FIELD, CV_8UC3, Scalar(0, 0, 0));
    attractive_force.Init(Kattr, a, b, 300);
    repulsive_force.Init(Krepl, d0);
}

void DrawTriangle(Mat &frame, const float center_x, const float center_y, const float radius, const Scalar color)
{
    int x1 = center_x - radius;
    int y1 = center_y - radius;
    int x2 = center_x;
    int y2 = center_y + radius;
    int x3 = center_x + radius;
    int y3 = center_y - radius;

    // Define the vertices of the triangle
    vector<Point> vertices = {Point(x1, y1), Point(x2, y2), Point(x3, y3)};

    // Fill the triangle with the specified color
    fillPoly(frame, vector<vector<Point>>{vertices}, color);

    // Draw the outline of the triangle
    line(frame, Point(x1, y1), Point(x2, y2), Scalar(0, 0, 0), 1);
    line(frame, Point(x2, y2), Point(x3, y3), Scalar(0, 0, 0), 1);
    line(frame, Point(x3, y3), Point(x1, y1), Scalar(0, 0, 0), 1);
}