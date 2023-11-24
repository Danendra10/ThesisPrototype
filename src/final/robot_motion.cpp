#include <simple_example/final/robot_motion.hpp>

vector<Point2f> potential_force_vect;
int points_per_row = (X_FIELD - 100) / 10;

void PotentialFieldGrapher()
{
    for (int x = 50; x < X_FIELD - 50; x += 10)
    {
        for (int y = 50; y < Y_FIELD - 50; y += 10)
        {
            float f_attr_x;
            float f_attr_y;
            AttractiveForce(x, robot_target_x, y, robot_target_y, 2, 50, 100, f_attr_x, f_attr_y);
            f_attr_x = (f_attr_x / 200) * 5;
            f_attr_y = (f_attr_y / 200) * 5;

            float f_x_rep_total = 0, f_y_rep_total = 0;

            for (size_t i = 0; i < sizeof(enemy) / sizeof(enemy[0]); i++)
            {
                float f_rep_x;
                float f_rep_y;
                RepulsiveForce(x, enemy[i].pose_x, y, enemy[i].pose_y, 100000000, 400, f_rep_x, f_rep_y);
                f_x_rep_total += f_rep_x * -1;
                f_y_rep_total += f_rep_y * -1;
            }

            float f_x_total = -f_attr_x + f_x_rep_total;
            float f_y_total = -f_attr_y + f_y_rep_total;

            potential_force_vect.push_back(Point2f(f_x_total, f_y_total));
        }
    }

    for (size_t i = 0; i < potential_force_vect.size(); i++)
    {
        if (isinf(potential_force_vect[i].x) || isnan(potential_force_vect[i].x) ||
            isinf(potential_force_vect[i].y) || isnan(potential_force_vect[i].y))
        {
            potential_force_vect[i].x = 0;
            potential_force_vect[i].y = 0;
            continue;
        }
        float magnitude = sqrt(potential_force_vect[i].x * potential_force_vect[i].x + potential_force_vect[i].y * potential_force_vect[i].y);
        if (isinf(magnitude) || isnan(magnitude))
        {
            printf("x %f y %f mag %f\n", potential_force_vect[i].x, potential_force_vect[i].y, magnitude);
        }
        // printf("x %f y %f mag %f\n", potential_force_vect[i].x, potential_force_vect[i].y, magnitude);
        // To avoid division by zero, check if magnitude is not zero
        if (magnitude != 0)
        {
            potential_force_vect[i].x /= magnitude;
            potential_force_vect[i].y /= magnitude;
        }
        else
        {
            potential_force_vect[i].x = 0;
            potential_force_vect[i].y = 0;
        }
    }
}
int main()
{
    PotentialFieldGrapher();
    namedWindow("Slider");
    createTrackbar("X enemy 1", "Slider", &x_enemy_1, X_FIELD, OnTrackbarCallback);
    createTrackbar("Y enemy 1", "Slider", &y_enemy_1, Y_FIELD, OnTrackbarCallback);
    createTrackbar("X enemy 2", "Slider", &x_enemy_2, X_FIELD, OnTrackbarCallback);
    createTrackbar("Y enemy 2", "Slider", &y_enemy_2, Y_FIELD, OnTrackbarCallback);
    createTrackbar("X target", "Slider", &robot_target_x, X_FIELD, OnTrackbarCallback);
    createTrackbar("Y target", "Slider", &robot_target_y, Y_FIELD, OnTrackbarCallback);
    namedWindow("Data Frame");
    while (true)
    {
        MainGameCallback();

        auto end = steady_clock::now();

        // Cast the duration to floating-point milliseconds
        duration<float, milli> time_diff_ms = end - start;

        // write the time difference to the DataFrame
        Mat frame = Mat(500, 500, CV_8UC3, Scalar(255, 255, 255));
        // putText(frame, "Time taken: " + to_string(time_diff_ms.count()), Point(10, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2, LINE_AA);

        // printf("portential vec size: %lu\n", potential_force_vect.size());
        imshow("Data Frame", frame);

        // Use %f to print floating-point values
        // logger_instance.Log(logger::YELLOW, "Time taken: %.3f ms", time_diff_ms.count());
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

    // MainAlgorithm();
    DrawOurRobot();

    DrawTarget();

    for (int x = 50; x < X_FIELD - 50; x += 10)
    {
        for (int y = 50; y < Y_FIELD - 50; y += 10)
        {
            // if (x > 1230 && y > 830)
            //     printf("x %d y %d idx %d\n", x, y, (x - 50) / 10 * 45 + (y - 50) / 10);
            // line(main_frame, Point(x, y), Point(x + potential_force_vect[(x - 50) / 10 * 45 + (y - 50) / 10].x * 10, y + potential_force_vect[(x - 50) / 10 * 45 + (y - 50) / 10].y * 10), CV_BLACK, 1, 8, 0);

            int row_index = (y - 50) / 10;
            int col_index = (x - 50) / 10;
            int index = row_index * points_per_row + col_index;
            // printf("x %d y %d idx %d\n", x, y, index);
            line(main_frame, Point(x, y), Point(x + potential_force_vect[index].x * 30, y + potential_force_vect[index].y * 30), CV_BLACK, 1, 8, 0);
        }
    }

    // for (size_t i = 0; i < attacker_vec_movement_points.size(); i++)
    // {
    //     circle(main_frame, Point(attacker_vec_movement_points[i].x, attacker_vec_movement_points[i].y), 5, CV_BLACK, -1, 8, 0);
    // }
    // if (sqrt(pow(robot_target_x - robot[0].pose_x, 2) + pow(robot_target_y - robot[0].pose_y, 2)) < 30)
    // {
    //     // robot_target_x = rand() % X_FIELD;
    //     // robot_target_y = rand() % Y_FIELD;

    //     // pause all display
    //     waitKey(0);
    // }
    Display();
}

void MainAlgorithm()
{
    float f_attr_x;
    float f_attr_y;
    AttractiveForce(robot[0].pose_x, robot_target_x, robot[0].pose_y, robot_target_y, 2, 50, 100, f_attr_x, f_attr_y);
    f_attr_x = (f_attr_x / 200) * 5;
    f_attr_y = (f_attr_y / 200) * 5;

    float f_x_rep_total = 0, f_y_rep_total = 0;

    for (size_t i = 0; i < sizeof(enemy) / sizeof(enemy[0]); i++)
    {
        float f_rep_x;
        float f_rep_y;
        RepulsiveForce(robot[0].pose_x, enemy[i].pose_x, robot[0].pose_y, enemy[i].pose_y, 100000000, 400, f_rep_x, f_rep_y);
        f_x_rep_total += f_rep_x * -1;
        f_y_rep_total += f_rep_y * -1;
    }

    float f_x_total = -f_attr_x + f_x_rep_total;
    float f_y_total = -f_attr_y + f_y_rep_total;

    robot[0].pose_x += f_x_total;
    robot[0].pose_y += f_y_total;

    logger_instance.Log(logger::RED, "Robot Position: %f %f || target: %d %d", robot[0].pose_x, robot[0].pose_y, robot_target_x, robot_target_y);

    attacker_movement_points.x = robot[0].pose_x;
    attacker_movement_points.y = robot[0].pose_y;

    attacker_vec_movement_points.push_back(attacker_movement_points);
}