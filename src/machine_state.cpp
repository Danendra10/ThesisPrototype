#include <simple_example/game_algorithm.hpp>

int main()
{

    if (Initialize() == -1)
        logger_instance.Log(logger::YELLOW, "Initialization failed");

    // This variable will control the UI loop
    std::atomic<bool> finished{false};

    // This variable will hold UI tasks to be executed on the main thread
    std::queue<std::function<void()>> ui_tasks;
    std::mutex ui_tasks_mutex;

    // Set up your timer on a separate thread
    Timer timer([&]
                {
        // Enqueue a task to call Display on the main thread
        std::lock_guard<std::mutex> lock(ui_tasks_mutex);
        ui_tasks.push([]{ MainGameCallback(); }); },
                1000 / 60);

    // Start the timer
    timer.start();

    // Run the UI processing loop on the main thread
    while (!finished)
    {
        std::unique_lock<std::mutex> lock(ui_tasks_mutex);
        if (!ui_tasks.empty())
        {
            auto task = std::move(ui_tasks.front());
            ui_tasks.pop();
            lock.unlock();
            task();
        }
        else
        {
            // If there's no UI task to execute, release the lock and allow the thread
            // to sleep for a short duration to prevent busy waiting
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // Clean up and stop the timer before exiting the program
    timer.stop();

    return 0;
}
int Initialize()
{
    try
    {
        attractive_force.Init(Kattr, a, b, maxVel);
        repulsive_force.Init(Krepl, d0);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
}

void MainGameCallback()
{
    KeyboardHandler();
    ClearFrame();
    DrawField();

    MainAlogirthm();
    DrawOurRobot();
    DrawBall();

    // logger_instance.Log(logger::RED, "%f", sqrt(pow(robot[0].pose_x - robot[0].last_pose_x, 2) + pow(robot[0].pose_y - robot[0].last_pose_y, 2)));
    // logger_instance.Log(logger::GREEN, "X: %f, Y: %f || Vel X: %f, Y: %f", robot[0].pose_x, robot[0].pose_y, robot_vel.x, robot_vel.y);

    Display();
}

void MainAlogirthm()
{
    static uint8_t get_target = 0;
    // last time held the ball

    static chrono::steady_clock::time_point last_time_held_ball;

    if (!robot[0].is_holding_ball)
    {
        CountRobotTarget(robot_target);
        CountDesiredVel();
        if (sqrt(ball.pose_x - robot[0].pose_x) + sqrt(ball.pose_y - robot[0].pose_y) < 5)
        {
            robot[0].is_holding_ball = true;
            num_robot_holding_ball = 0;
            UpdateRobotLastPose(0);
            last_time_held_ball = chrono::steady_clock::now();
        }
    }
    else
    {
        if (sqrt(pow(robot[0].pose_x - robot[0].last_pose_x, 2) + pow(robot[0].pose_y - robot[0].last_pose_y, 2)) < 300 && !get_target)
        {
            logger_instance.Log(logger::YELLOW, "Holding ball %f", sqrt(pow(robot[0].pose_x - robot[0].last_pose_x, 2) + pow(robot[0].pose_y - robot[0].last_pose_y, 2)));
            CountRobotTarget(robot_target);
            get_target = 1;
        }
        else
        {
            auto current_time = std::chrono::steady_clock::now();
            auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_time_held_ball);
            if (time_diff.count() > 5)
            {
                robot[0].is_passing = true; // Set passing to true if more than 5 seconds have passed
            }
            logger_instance.Log(logger::CYAN, "time diff: %d || passing: %d", time_diff.count(), robot[0].is_passing);

            for (const auto &obstacle : obstacles)
            {

                if (IsCircleLineIntersecting(robot[0].pose_x, robot[0].pose_y, goal_pose.x, goal_pose.y, obstacle.x, obstacle.y, 20))
                {
                    line(main_frame, Point(robot[0].pose_x, robot[0].pose_y), Point(goal_pose.x, goal_pose.y), CV_RED, 2);
                    line(main_frame, Point(robot[0].pose_x, robot[0].pose_y), Point(obstacle.x, obstacle.y), CV_MAGENTA, 2);
                    robot[0].is_kicking = false;
                    logger_instance.Log(logger::RED, "Kicking %d", robot[0].is_kicking);
                }
                else
                {
                    robot[0].is_kicking = true;
                    logger_instance.Log(logger::YELLOW, "Kicking %d", robot[0].is_kicking);
                }
            }
        }
        CountDesiredVel();
    }
}

void UpdateRobotLastPose(uint8_t num)
{
    robot[num].last_pose_x = robot[num].pose_x;
    robot[num].last_pose_y = robot[num].pose_y;
    robot[num].last_pose_th = robot[num].pose_th;
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
    circle(main_frame, Point(robot[0].pose_x, robot[0].pose_y), 20, CV_MAGENTA, -1);
    // circle(main_frame, Point(robot[1].pose_x, robot[1].pose_y), 20, CV_MAGENTA, -1);

    for (const auto &obstacle : obstacles)
    {
        circle(main_frame, obstacle, 20, Scalar(0, 0, 255), -1); // Draw obstacles
        putText(main_frame, "x = " + std::to_string(obstacle.x) + " y = " + std::to_string(obstacle.y), obstacle, 1, 1, CV_WHITE);
    }
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

void CountDesiredVel()
{
    float f_x_attr, f_y_attr;
    attractive_force.Update(robot[0].pose_x, robot[0].pose_y, robot_target.x, robot_target.y, maxVel, f_x_attr, f_y_attr);
    printf("Attr: %f %f\n", f_x_attr, f_y_attr);
    float f_x_rep_total = 0, f_y_rep_total = 0;

    // logger_instance.Log(logger::YELLOW, "X %f Y %f IsOutsideRobotMaxRadius: %f", x_curr, y_curr, sqrt(pow(x_curr - robot_pose.x, 2) + pow(y_curr - robot_pose.y, 2)));
    for (const auto &obstacle : obstacles)
    {
        // when the current
        if (sqrt((robot[0].pose_x - obstacle.x) * (robot[0].pose_x - obstacle.x) + (robot[0].pose_y - obstacle.y) * (robot[0].pose_y - obstacle.y)) < exclude_radius_around_obstacles ||
            CheckLineCircleIntersection2(robot[0].pose_x, robot[0].pose_y, robot[0].pose_x, robot[0].pose_y, obstacle.x, obstacle.y, exclude_radius_around_obstacles))
        {
            break;
        }

        float f_x_rep, f_y_rep;
        repulsive_force.Update(robot[0].pose_x, robot[0].pose_y, obstacle.x, obstacle.y, f_x_rep, f_y_rep);
        f_x_rep_total += f_x_rep * -1;
        f_y_rep_total += f_y_rep * -1;
    }

    float f_x_total = f_x_attr + f_x_rep_total;
    float f_y_total = f_y_attr + f_y_rep_total;
    logger_instance.Log(logger::BLUE, "Vel X: %f, Vel Y: %f", f_x_total, f_y_total);
    if (abs(robot[0].pose_x - robot_target.x) < 10 && abs(robot[0].pose_y - robot_target.y) < 10)
    {
        robot_vel.x = 0;
        robot_vel.y = 0;
        return;
    }
    robot[0].pose_x += f_x_total;
    robot[0].pose_y += f_y_total;
}

void CountRobotTarget(Point2f &point)
{
    uint8_t robot_able_to_shoot = 0;

    if (!robot[0].is_holding_ball)
    {
        point.x = ball.pose_x;
        point.y = ball.pose_y;
        return;
    }

    if (robot[0].pose_x >= X_FIELD_3_4)
        robot_able_to_shoot = 1;

    if (robot_able_to_shoot)
        return;
    float least_force_x = __FLT_MAX__;
    float least_force_y = __FLT_MAX__;
    float least_force_x_idx = __FLT_MAX__;
    float least_force_y_idx = __FLT_MAX__;

    float target_x = (!robot[0].is_holding_ball ? ball.pose_x : goal_pose.x);
    float target_y = (!robot[0].is_holding_ball ? ball.pose_y : goal_pose.y);

    for (int i = 0; i <= X_FIELD; i += stepSize)
    {
        for (int j = 0; j <= Y_FIELD; j += stepSize)
        {
            // Current position
            float x_curr = i;
            float y_curr = j;

            if (IsOutsideRobotMaxRadius(x_curr, y_curr, robot[0].pose_x, robot[0].pose_y, max_robot_movement_radius))
                continue;

            if (robot[0].pose_x <= X_FIELD_3_4 && x_curr <= robot[0].pose_x)
                continue;
            if (robot[0].pose_x <= X_FIELD_1_4 && x_curr <= X_FIELD_1_4)
                continue;

            // Calculate attractive force
            float f_x_attr, f_y_attr;
            attractive_force.Update(x_curr, y_curr, target_x, target_y, maxVel, f_x_attr, f_y_attr);

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
                    CheckLineCircleIntersection2(x_curr, y_curr, robot[0].pose_x, robot[0].pose_y, obstacle.x, obstacle.y, exclude_radius_around_obstacles))
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

    circle(main_frame, Point2d(least_force_x_idx, least_force_y_idx), 10, CV_WHITE, -1);
    line(main_frame, Point2d(robot[0].pose_x, robot[0].pose_y), Point2d(least_force_x_idx, least_force_y_idx), CV_WHITE, 1);

    point.x = least_force_x_idx;
    point.y = least_force_y_idx;
}

void processUITasks()
{
    while (!finished)
    {
        std::unique_lock<std::mutex> lock(ui_tasks_mutex);
        ui_tasks_cv.wait(lock, []
                         { return !ui_tasks.empty() || finished; });

        while (!ui_tasks.empty())
        {
            auto task = std::move(ui_tasks.front());
            ui_tasks.pop();
            lock.unlock();
            task();
            lock.lock();
        }
    }
}

void enqueueUITask(std::function<void()> task)
{
    std::lock_guard<std::mutex> lock(ui_tasks_mutex);
    ui_tasks.push(std::move(task));
    ui_tasks_cv.notify_one();
}