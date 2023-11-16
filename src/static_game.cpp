#include <simple_example/static_game.hpp>

int main()
{
    if (Initialize() == -1)
        logger_instance.Log(logger::YELLOW, "Initialization failed");

    while (true)
    {
        StaticGame();
    }

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

void StaticGame()
{
    KeyboardHandler();

    DrawField();

    DrawBall();

    // Own Robot
    DrawRobot(robot[0].pose_x, robot[0].pose_y, robot[0].pose_th, CV_MAGENTA);
    DrawRobot(robot[1].pose_x, robot[1].pose_y, robot[1].pose_th, CV_MAGENTA);

    // Enemy
    DrawRobot(enemy[0].pose_x, enemy[0].pose_y, enemy[0].pose_th, CV_BLACK);
    DrawRobot(enemy[1].pose_x, enemy[1].pose_y, enemy[1].pose_th, CV_BLACK);

    // Legend
    WriteLegend();

    Display();
}
