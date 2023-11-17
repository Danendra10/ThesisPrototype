#ifndef MAIN_GAME_HPP_
#define MAIN_GAME_HPP_

#pragma once

#include <math/vec_attr.hpp>
#include <math/vec_repl.hpp>
#include <utils/utils.hpp>
#include <logger/logger.hpp>
#include <simple_example/machine_state.hpp>
#include <utils/track_keyboard.hpp>
#include <utils/timer.hpp>
#include <utils/entity.hpp>
#include <utils/pid.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//---> Macros for colors
#define CV_RED Scalar(0, 0, 255)
#define CV_GREEN Scalar(0, 255, 0)
#define CV_BLUE Scalar(255, 0, 0)
#define CV_YELLOW Scalar(0, 255, 255)
#define CV_MAGENTA Scalar(255, 0, 255)
#define CV_CYAN Scalar(255, 255, 0)
#define CV_WHITE Scalar(255, 255, 255)
#define CV_BLACK Scalar(0, 0, 0)

//---> Macros for roles
#define ATTACKER 1
#define ASSIST 2
#define ENEMY 10

//---> Macros for conversion
#ifndef Cm2PxX
#define Cm2PxX(x_) ((x_))
#endif
#ifndef Cm2PxY
#define Cm2PxY(y_) (1300 - (y_))
#endif
#ifndef Px2CmX
#define Px2CmX(x_) ((x_)-50)
#endif
#ifndef Px2CmY
#define Px2CmY(y_) (1250 - (y_))
#endif
#ifndef DEG2RAD
#define DEG2RAD 0.017452925
#endif
#ifndef RAD2DEG
#define RAD2DEG 57.295780
#endif
#ifndef DIV180
#define DIV180 0.005555556
#endif
#ifndef sqr
#define sqr(a) ((a) * (a))
#endif
#ifndef dmin
#define dmin(a, b) ((a) <= (b) ? (a) : (b))
#endif
#ifndef dmax
#define dmax(a, b) ((a) >= (b) ? (a) : (b))
#endif

//---> Macros for fields
#define X_FIELD 900
#define Y_FIELD 1300
#define X_FIELD_1_2 (X_FIELD * 0.5)
#define Y_FIELD_1_2 (Y_FIELD * 0.5)
#define X_FIELD_1_4 (X_FIELD * 0.25)
#define Y_FIELD_1_4 (Y_FIELD * 0.25)
#define X_FIELD_3_4 (X_FIELD * 0.75)
#define Y_FIELD_3_4 (Y_FIELD * 0.75)

//---> Extern Variables
extern uint8_t game_status;
extern uint8_t robot_base_action;

//---> Timer like ros timer
std::queue<std::function<void()>> ui_tasks;
std::mutex ui_tasks_mutex;
std::condition_variable ui_tasks_cv;
std::atomic<bool> finished{false};

//---> Logger
logger::Logger logger_instance;

//---> StateMachine
MachineState main_state;
MachineState sub_state;

//---> Entities
int8_t num_robot_handling_ball = -1;
int8_t num_enemy_handling_ball = -1;
RobotEntity robot[2] = {
    RobotEntity(ATTACKER, X_FIELD_1_4, Y_FIELD_1_4, 90),
    RobotEntity(ASSIST, X_FIELD_3_4, Y_FIELD_1_4, 90)};

RobotEntity enemy[2] = {
    RobotEntity(ENEMY, X_FIELD_1_4, Y_FIELD_3_4, -90, 0),
    RobotEntity(ENEMY, X_FIELD_3_4, Y_FIELD_3_4, -90, 0)};

BallEntity ball = {X_FIELD_1_2, Y_FIELD_1_2};

Point2f goal_pose(X_FIELD, Y_FIELD_1_2);

int8_t attacker_number = -1;
int8_t assist_number = -1;

//---> Matrixes
Mat main_frame = Mat(Y_FIELD, X_FIELD, CV_8UC3, Scalar(0, 0, 0));

//---> Forces
Attractive::Force attractive_force;
Repulsive::Force repulsive_force;

//---> Tuningable Parameters
int step_size = 17;
int max_vel = 2;
int max_vel_rot = 2;
int k_attr = 1;
int k_repl = 89381793;
int d0 = 100;
int a = 1;
int b = 1;
int _speed = 77;
uint8_t exclude_radius_around_obstacles = 100;
uint8_t robot_radius = 20;
uint8_t restricted_min_passing_areas_friend = 50;
uint16_t restricted_max_passing_areas_friend = 300;
uint16_t max_robot_movement_radius = 300;

//---> Prototypes

//--------------------------------- Initialization ----------------------------------//
int Initialize();

//-------------------------------------- Loop --------------------------------------//

void MainGameCallback();
void MainMachineState();

//-------------------------------------- Drawers --------------------------------------//
void DrawRobot(float x_, float y_, float th_, Scalar color_)
{
    circle(main_frame, Point2f(Cm2PxX(x_), Cm2PxY(y_)), 30, color_, 4);
    line(main_frame, Point2f(Cm2PxX(x_), Cm2PxY(y_)), Point(Cm2PxX(x_ + 35 * cos(th_ * DEG2RAD)), Cm2PxY(y_ + 35 * sin(th_ * DEG2RAD))), color_, 3);
}

void DrawBall()
{
    float pos_ball[2] = {ball.pose_x, ball.pose_y};
    circle(main_frame, Point2f(pos_ball[0], pos_ball[1]), 10, CV_RED, -1);
}

void Display()
{
    imshow("main_frame", main_frame);
    if (waitKey(1) == 'q')
    {
        exit(0);
    }
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
    int penalty_area_top_x = (X_FIELD / 2) - (penalty_area_height / 2);
    int goal_area_top_x = (X_FIELD / 2) - (goal_area_height / 2);

    rectangle(main_frame, Rect(Point2i(0, 0), Point2i(X_FIELD, Y_FIELD)), CV_GREEN, -1);
    // create outer line with distance of 50 from the outer box
    rectangle(main_frame, Rect(Point2i(50, 50), Point2i(X_FIELD - 50, Y_FIELD - 50)), CV_WHITE, 5);
    // center line
    line(main_frame, Point(50, Y_FIELD_1_2), Point(X_FIELD - 50, Y_FIELD_1_2), CV_WHITE, 5);
    // center circle
    circle(main_frame, Point(X_FIELD_1_2, Y_FIELD_1_2), 130, CV_WHITE, 5);
    circle(main_frame, Point(X_FIELD_1_2, Y_FIELD_1_2), 10, CV_WHITE, -1);
    // Draw penalty and goal areas for both sides
    rectangle(main_frame, Point(penalty_area_top_x, 50), Point(penalty_area_top_x + penalty_area_height, 50 + penalty_area_width), CV_WHITE, 5);
    rectangle(main_frame, Point(penalty_area_top_x, Y_FIELD - 50 - penalty_area_width), Point(penalty_area_top_x + penalty_area_height, Y_FIELD - 50), CV_WHITE, 5);

    rectangle(main_frame, Point(goal_area_top_x, 50), Point(goal_area_top_x + goal_area_height, 50 + goal_area_width), CV_WHITE, 5);
    rectangle(main_frame, Point(goal_area_top_x, Y_FIELD - 50 - goal_area_width), Point(goal_area_top_x + goal_area_height, Y_FIELD - 50), CV_WHITE, 5);

    // Penalty spots
    circle(main_frame, Point(50 + penalty_spot_distance, Y_FIELD / 2), 5, CV_WHITE, -1);
    circle(main_frame, Point(X_FIELD - 50 - penalty_spot_distance, Y_FIELD / 2), 5, CV_WHITE, -1);

    // Corner arcs
    ellipse(main_frame, Point(50, 50), Size(corner_arc_radius, corner_arc_radius), 0, 0, 90, CV_WHITE, 5);
    ellipse(main_frame, Point(50, Y_FIELD - 50), Size(corner_arc_radius, corner_arc_radius), 180, 90, 180, CV_WHITE, 5);
    ellipse(main_frame, Point(X_FIELD - 50, 50), Size(corner_arc_radius, corner_arc_radius), 90, 90, 0, CV_WHITE, 5);
    ellipse(main_frame, Point(X_FIELD - 50, Y_FIELD - 50), Size(corner_arc_radius, corner_arc_radius), 0, 180, 270, CV_WHITE, 5);
}

//--------------------------------- Calculation ----------------------------------//
float RobotAngleToBall(const RobotEntity robot, const BallEntity ball)
{
    float dx = ball.pose_x - robot.pose_x;
    float dy = ball.pose_y - robot.pose_y;
    return atan2(dy, dx) * RAD2DEG;
}

float RobotAngleToPoint(const RobotEntity robot, float x, float y)
{
    float dx = x - robot.pose_x;
    float dy = y - robot.pose_y;
    return atan2(dy, dx) * RAD2DEG;
}

static array<float, 2> MidPointBetweenTwoPoint(float x1, float y1, float x2, float y2)
{
    array<float, 2> midpoint;

    midpoint[0] = (x1 + x2) / 2.0; // Calculate the x-coordinate of the midpoint
    midpoint[1] = (y1 + y2) / 2.0; // Calculate the y-coordinate of the midpoint

    return midpoint;
}

/**
 * Find a midpoint of a triangle
 * (x1, y1)     (x2,y2)
 * o-----------o
 * \          /
 *  \   o   /
 *   \    /
 *    \ /
 *     o
 *      (x3,y3)
 * @param x1 x coordinate of point 1
 * @param y1 y coordinate of point 1
 * @param x2 x coordinate of point 2
 * @param y2 y coordinate of point 2
 * @param x3 x coordinate of point 3
 * @param y3 y coordinate of point 3
 */
static array<float, 2> MidPointTriangle(float x1, float y1, float x2, float y2, float x3, float y3)
{
    std::array<float, 2> midpoint;
    midpoint[0] = (x1 + x2 + x3) / 3.0;
    midpoint[1] = (y1 + y2 + y3) / 3.0;
    return midpoint;
}

float AngleDifference(float attacker_th, float attacker_x, float attacker_y, float assist_x, float assist_y)
{
    // Calculate the angle from the attacker to the assistant
    float angle_to_assistant = atan2(assist_y - attacker_y, assist_x - attacker_x);

    // Calculate the raw difference in the two angles
    float raw_diff = attacker_th - angle_to_assistant;

    // Normalize the difference to be within the range -π to π
    float normalized_diff = fmod(raw_diff + M_PI, 2 * M_PI) - M_PI;

    // Return the absolute value of the normalized difference
    return fabs(normalized_diff);
}

/**
 * @return the position to go
 */
static array<float, 3> PositionToGo(const RobotEntity robot, RobotEntity friend_, const PRobotEntity_t obstacles, const BallEntity ball, uint8_t step_size, Attractive::Force attractive_force, Repulsive::Force repuslive_force, uint16_t flags = approach_ball)
{
    std::array<float, 3> position = {0, 0, 0};

    std::array<float, 2> target;
    target[0] = ((flags & approach_ball) == approach_ball || (flags & approach_ball_preparation) == approach_ball_preparation) ? Cm2PxX(ball.pose_x) : 800;
    target[1] = ((flags & approach_ball) == approach_ball || (flags & approach_ball_preparation) == approach_ball_preparation) ? Cm2PxY(ball.pose_y) : 1200;

    if ((flags & avoid_ball) == avoid_ball)
    {
        float side_point[2] = {(Cm2PxX(robot.pose_x) <= X_FIELD_1_2) ? 0 : static_cast<float>(X_FIELD), static_cast<float>(Y_FIELD_1_2)};
        std::array<float, 2> target_temp = MidPointTriangle(Cm2PxX(robot.pose_x), Cm2PxY(robot.pose_y), Cm2PxX(ball.pose_x), Cm2PxY(ball.pose_y), side_point[0], side_point[1]);
        target[0] = target_temp[0];
        target[1] = target_temp[1];
    }

    static float highest_force_x = 0;
    static float highest_force_y = 0;

    for (uint16_t x = 0; x < X_FIELD; x += step_size)
    {
        for (uint16_t y = 0; y < Y_FIELD; y += step_size)
        {
            if (((flags & approach_ball_preparation) == approach_ball_preparation || (flags & avoid_ball) == avoid_ball) && y <= Y_FIELD_1_2)
                continue;
            if (friend_.pose_x <= X_FIELD_1_2 && x <= X_FIELD_1_2)
                continue;
            if (friend_.pose_x > X_FIELD_1_2 && x > X_FIELD_1_2)
                continue;

            float f_x_attr, f_y_attr;
            attractive_force.Update(x, y, target[0], target[1], max_vel, f_x_attr, f_y_attr);

            float f_x_rep_total = 0, f_y_rep_total = 0;

            for (int i = 0; i < 2; i++)
            {

                float f_x_rep, f_y_rep;
                repulsive_force.Update(x, y, Cm2PxX(obstacles[i].pose_x), Cm2PxY(obstacles[i].pose_y), f_x_rep, f_y_rep);
                f_x_rep_total -= f_x_rep;
                f_y_rep_total -= f_y_rep;
            }
            // Total force
            float f_x_total = f_x_attr + f_x_rep_total;
            float f_y_total = f_y_attr + f_y_rep_total;
            if (abs(f_x_total) > abs(highest_force_x) || abs(f_y_total) > abs(highest_force_y))
            {
                highest_force_x = f_x_total;
                position[0] = x;
                highest_force_y = f_y_total;
                position[1] = y;
            }

            line(main_frame, Point(x, y), Point(x + f_x_total, y + f_y_total), Scalar(0, 0, 255), 10);
        }
    }

    position[2] = atan2(position[1] - robot.pose_y, position[0] - robot.pose_x) * RAD2DEG;

    line(main_frame, Point(Cm2PxX(robot.pose_x), Cm2PxY(robot.pose_y)), Point(position[0], position[1]), Scalar(255, 0, 0), 10);
    line(main_frame, Point(Cm2PxX(robot.pose_x), Cm2PxY(robot.pose_y)), Point(target[0], target[1]), Scalar(255, 0, 255), 10);
    circle(main_frame, Point(position[0], position[1]), 10, Scalar(0, 255, 0), -1);

    return position;
}

void CalcVelocity(RobotEntity &robot, array<float, 3> target, float max_vel_trans, float max_vel_ang, uint16_t flags = approach_ball)
{
    static PID_t angles_pid;
    PIDInit(&angles_pid, 4, 0, 0);

    float f_x_attr, f_y_attr;
    attractive_force.Update(robot.pose_x, robot.pose_y, target[0], target[1], max_vel_trans, f_x_attr, f_y_attr);
    float f_x_rep_total, f_y_rep_total;

    for (const auto &obstacle : enemy)
    {
        float f_x_rep, f_y_rep;
        repulsive_force.Update(robot.pose_x, robot.pose_y, obstacle.pose_x, obstacle.pose_y, f_x_rep, f_y_rep);
        f_x_rep_total -= f_x_rep;
        f_y_rep_total -= f_y_rep;
    }

    float f_x_total = f_x_attr + f_x_rep_total;
    float f_y_total = f_y_attr + f_y_rep_total;

    /* Position error */
    float position_error = sqrt(pow(target[0] - Cm2PxX(robot.pose_x), 2) + pow(target[1] - Cm2PxY(robot.pose_y), 2));
    /* Theta error */
    float theta_error = target[2] - robot.pose_th;
    logger_instance.Log(logger::GREEN, "Target: %f %f %f", target[0], target[1], target[2]);

    while (theta_error < -180)
        theta_error += 360;
    while (theta_error > 180)
        theta_error -= 360;

    float th_output = PIDCalculate(&angles_pid, theta_error, max_vel_ang);

    if (fabs(position_error) < 20 && fabs(th_output) < 5)
    {
        robot.vel_x = 0;
        robot.vel_y = 0;
        robot.vel_th = 0;
        return;
    }

    while (abs(th_output) > max_vel_ang || isnan(th_output))
    {
        th_output = max_vel_ang * (th_output > 0 ? 1 : -1);
    }

    while (abs(f_x_total) > max_vel_trans || isnan(f_x_total))
    {
        f_x_total = max_vel_trans * (f_x_total > 0 ? 1 : -1);
    }

    while (abs(f_y_total) > max_vel_trans || isnan(f_y_total))
    {
        f_y_total = max_vel_trans * (f_y_total > 0 ? 1 : -1);
    }

    robot.vel_x = f_x_total;
    robot.vel_y = f_y_total;
    robot.vel_th = th_output;

    // logger_instance.Log(logger::YELLOW, "f_x_total: %f, f_y_total: %f, th_output: %f", f_x_total, f_y_total, th_output);
}

void MoveRobot()
{
    for (int i = 0; i < 2; i++)
    {
        robot[i].pose_x += robot[i].vel_x;
        robot[i].pose_y += robot[i].vel_y;
        robot[i].pose_th += robot[i].vel_th;
    }
}

// Utility function to compare floating point numbers with a tolerance
bool areFloatsEqual(float a, float b, float tolerance = 0.001)
{
    return fabs(a - b) < tolerance;
}

void CheckRobotHoldingBall()
{
    // Check each robot
    for (int i = 0; i < 2; i++)
    {
        // Calculate the distance between the robot and the ball
        float distance = sqrt(pow(Cm2PxX(robot[i].pose_x) - Cm2PxX(ball.pose_x), 2) + pow(Cm2PxY(robot[i].pose_y) - Cm2PxY(ball.pose_y), 2));
        // robot[i].is_holding_ball = 0;

        // If distance is less than 30, update num_robot_handling_ball
        if (distance < 30)
        {
            num_robot_handling_ball = i;
            robot[i].is_holding_ball = 1;
            break; // Assuming only one robot can handle the ball at a time
        }
    }

    // If no robot is handling the ball, check each enemy
    if (num_robot_handling_ball == -1)
    {
        for (int i = 0; i < 2; i++)
        {
            // Calculate the distance between the enemy and the ball
            float distance = sqrt(pow(Cm2PxX(enemy[i].pose_x) - Cm2PxX(ball.pose_x), 2) + pow(Cm2PxY(enemy[i].pose_y) - Cm2PxY(ball.pose_y), 2));
            // If distance is less than 30, update num_enemy_handling_ball
            if (distance < 30)
            {
                num_enemy_handling_ball = i;
                break; // Assuming only one enemy can handle the ball at a time
            }
        }
    }
}

void UpdateBallPosWhenHeld()
{
    if (num_robot_handling_ball != -1)
    {
        logger_instance.Log(logger::GREEN, "Robot %d is handling the ball", num_robot_handling_ball);
        ball.pose_x = Cm2PxX(robot[num_robot_handling_ball].pose_x + 35 * cos(robot[num_robot_handling_ball].pose_th * DEG2RAD));
        ball.pose_y = Cm2PxY(robot[num_robot_handling_ball].pose_y + 35 * sin(robot[num_robot_handling_ball].pose_th * DEG2RAD));
    }
    else if (num_enemy_handling_ball != -1)
    {
        ball.pose_x = Cm2PxX(enemy[num_enemy_handling_ball].pose_x + 35 * cos(enemy[num_enemy_handling_ball].pose_th * DEG2RAD));
        ball.pose_y = Cm2PxY(enemy[num_enemy_handling_ball].pose_y + 35 * sin(enemy[num_enemy_handling_ball].pose_th * DEG2RAD));
    }
}

//--------------------------------- Game ----------------------------------//
void AttackerAlgorithm(uint16_t flags);

void KickBall(uint16_t force)
{
    // Assuming pose_th is in degrees and needs to be converted to radians
    float kick_direction_rad = robot[num_robot_handling_ball].pose_th * DEG2RAD;
    logger_instance.Log(logger::GREEN, "KickBall: %f", kick_direction_rad);

    // Update the ball's position based on the force and direction
    ball.pose_x += force * cos(kick_direction_rad);
    ball.pose_y += force * sin(kick_direction_rad);
}
#endif