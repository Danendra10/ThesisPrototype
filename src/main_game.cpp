#include <simple_example/main_game.hpp>

int main()
{
    if (Initialize() == -1)
        logger_instance.Log(logger::YELLOW, "Initialization failed");

    while (true)
    {
        MainGameCallback();
    }

    // // This variable will control the UI loop
    // std::atomic<bool> finished{false};

    // // This variable will hold UI tasks to be executed on the main thread
    // std::queue<std::function<void()>> ui_tasks;
    // std::mutex ui_tasks_mutex;

    // // Set up your timer on a separate thread
    // Timer timer([&]
    //             {
    //     // Enqueue a task to call Display on the main thread
    //     std::lock_guard<std::mutex> lock(ui_tasks_mutex);
    //     ui_tasks.push([]{ MainGameCallback(); }); },
    //             1000 / 1000);

    // // Start the timer
    // timer.start();

    // // Run the UI processing loop on the main thread
    // while (!finished)
    // {
    //     std::unique_lock<std::mutex> lock(ui_tasks_mutex);
    //     if (!ui_tasks.empty())
    //     {
    //         auto task = std::move(ui_tasks.front());
    //         ui_tasks.pop();
    //         lock.unlock();
    //         task();
    //     }
    //     else
    //     {
    //         // If there's no UI task to execute, release the lock and allow the thread
    //         // to sleep for a short duration to prevent busy waiting
    //         lock.unlock();
    //         std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //     }
    // }

    // // Clean up and stop the timer before exiting the program
    // timer.stop();

    return 0;
}

int Initialize()
{
    try
    {
        attractive_force.Init(k_attr, a, b, 100);
        repulsive_force.Init(k_repl, d0);
        return 0;
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

    DrawBall();

    // Own Robot
    DrawRobot(robot[0].pose_x, robot[0].pose_y, robot[0].pose_th, CV_MAGENTA);
    DrawRobot(robot[1].pose_x, robot[1].pose_y, robot[1].pose_th, CV_MAGENTA);

    // Enemy
    DrawRobot(enemy[0].pose_x, enemy[0].pose_y, enemy[0].pose_th, CV_BLACK);
    DrawRobot(enemy[1].pose_x, enemy[1].pose_y, enemy[1].pose_th, CV_BLACK);

    MainMachineState();
    CheckRobotHoldingBall();
    UpdateBallPosWhenHeld();
    Display();
}

void MainMachineState()
{
    if (attacker_number == -1)
    {
        // calculate which robot is closest to ball
        float closest_distance = std::numeric_limits<float>::max();
        int closest_robot_index = -1;

        for (int i = 1; i <= 2; i++)
        {
            float distance = std::hypot(ball.pose_x - robot[i].pose_x, ball.pose_y - robot[i].pose_y);
            if (distance < closest_distance)
            {
                closest_distance = distance;
                closest_robot_index = i;
            }
        }
        attacker_number = closest_robot_index;
        assist_number = !attacker_number;
    }

    static array<float, 3> attacker_target;
    static array<float, 3> prev_attacker_target;
    static array<float, 3> assist_target;
    static array<float, 3> prev_assist_target;

    static float max_trans_vel = max_vel;
    static float max_rot_vel = max_vel_rot;
    // theta diffrence between attacker and assist
    float th_diff = AngleDifference(robot[attacker_number].pose_th, robot[attacker_number].pose_x, robot[attacker_number].pose_y, robot[assist_number].pose_x, robot[assist_number].pose_y);
    logger_instance.Log(logger::YELLOW, "TH DIFF: %f", th_diff * RAD2DEG);

    switch (game_status * robot_base_action)
    {
    case status_preparation_kickoff_home:
        if (prev_assist_target[0] == assist_target[0] || prev_assist_target[1] == assist_target[1])
        {
            assist_target = PositionToGo(robot[assist_number], robot[attacker_number], enemy, ball, step_size, attractive_force, repulsive_force, avoid_ball);
        }

        if (prev_attacker_target[0] == attacker_target[0] || prev_attacker_target[1] == attacker_target[1])
        {
            attacker_target = PositionToGo(robot[attacker_number], robot[assist_number], enemy, ball, step_size, attractive_force, repulsive_force, approach_ball_preparation);
        }

        logger_instance.Log(logger::YELLOW, "Attacker Target: %f, %f, %f", attacker_target[0], attacker_target[1], attacker_target[2]);
        logger_instance.Log(logger::YELLOW, "Assist Target: %f, %f, %f", assist_target[0], assist_target[1], assist_target[2]);

        assist_target[2] = RobotAngleToBall(robot[assist_number], ball);
        attacker_target[2] = RobotAngleToBall(robot[attacker_number], ball);

        break;

    case game_kickoff_home:
        if (robot[attacker_number].is_holding_ball)
        {
            logger_instance.Log(logger::YELLOW, "MASUK HOLDING BALL");
            attacker_target[0] = (robot[assist_number].pose_x);
            attacker_target[1] = (robot[assist_number].pose_y);
            attacker_target[2] = RobotAngleToPoint(robot[attacker_number], robot[assist_number].pose_x, robot[assist_number].pose_y);
            max_trans_vel = 0;
        }
        else
        {
            logger_instance.Log(logger::RED, "MASUK NOT HOLDING BALL");
            attacker_target[0] = Cm2PxX(ball.pose_x);
            attacker_target[1] = Cm2PxY(ball.pose_y);
            attacker_target[2] = RobotAngleToPoint(robot[attacker_number], robot[assist_number].pose_x, robot[assist_number].pose_y);
        }

        if (th_diff * RAD2DEG <= 40 && robot[attacker_number].is_holding_ball)
        {
            robot[attacker_number].is_kicking = 1;
            robot[attacker_number].is_holding_ball = 0;
            KickBall(100);
        }
        break;

    default:
        break;
    }

    if (attacker_target[0] < X_FIELD && attacker_target[0] > 0 &&
        attacker_target[1] < Y_FIELD && attacker_target[1] > 0)
        CalcVelocity(robot[attacker_number], attacker_target, max_trans_vel, max_rot_vel, approach_ball_preparation);

    if (assist_target[0] < X_FIELD && assist_target[0] > 0 &&
        assist_target[1] < Y_FIELD && assist_target[1] > 0)
        CalcVelocity(robot[assist_number], assist_target, max_trans_vel, max_rot_vel, avoid_ball);
    MoveRobot();
}

void PreparationAlgorithm(uint16_t flags)
{
    static array<float, 3> attacker_target;
    static float max_trans_vel = max_vel;
    static float max_rot_vel = max_vel_rot;
    float th_diff = fabs(robot[attacker_number].pose_th - robot[assist_number].pose_th);
    switch (flags)
    {
    case approach_ball_att:
        if (robot[attacker_number].is_holding_ball)
        {
            logger_instance.Log(logger::YELLOW, "MASUK HOLDING BALL");
            attacker_target[0] = (robot[assist_number].pose_x);
            attacker_target[1] = (robot[assist_number].pose_y);
            attacker_target[2] = RobotAngleToPoint(robot[attacker_number], robot[assist_number].pose_x, robot[assist_number].pose_y);
            max_trans_vel = 0;
        }
        else
        {
            logger_instance.Log(logger::RED, "MASUK NOT HOLDING BALL");
            attacker_target[0] = Cm2PxX(ball.pose_x);
            attacker_target[1] = Cm2PxY(ball.pose_y);
            attacker_target[2] = RobotAngleToPoint(robot[attacker_number], robot[assist_number].pose_x, robot[assist_number].pose_y);
        }

        // logger_instance.Log(logger::RED, "%f %f || %f", robot[attacker_number].pose_th, robot[assist_number].pose_th, fabs(robot[attacker_number].pose_th - robot[assist_number].pose_th));
        // if (th_diff >= 170 ||
        //     th_diff <= 190)
        //     flags = pass_ball_att;

        CalcVelocity(robot[attacker_number], attacker_target, max_trans_vel, max_rot_vel, approach_ball_preparation);
        MoveRobot();
        break;

    // case pass_ball_att:
    //     logger_instance.Log(logger::CYAN, "MASUK PASS BALL");
    //     break;
    default:
        break;
    }
}