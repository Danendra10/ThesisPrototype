#ifndef STATIC_GAME_HPP_
#define STATIC_GAME_HPP_

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

//---> Logger
logger::Logger logger_instance;

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

//--------------------------------- Initialization ----------------------------------//
int Initialize();

//-------------------------------------- Loop --------------------------------------//
void StaticGame();

//-------------------------------------- Drawers --------------------------------------//
void DrawRobot(float x_, float y_, float th_, Scalar color_)
{
    circle(main_frame, Point2f(Cm2PxX(x_), Cm2PxY(y_)), 30, color_, 4);
    line(main_frame, Point2f(Cm2PxX(x_), Cm2PxY(y_)), Point(Cm2PxX(x_ + 35 * cos(th_ * DEG2RAD)), Cm2PxY(y_ + 35 * sin(th_ * DEG2RAD))), color_, 3);
}

void WriteLegend()
{
    putText(main_frame, "Enemy", Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, CV_BLACK, 2);
    putText(main_frame, "Own Robot", Point(10, 60), FONT_HERSHEY_SIMPLEX, 1, CV_MAGENTA, 2);
    putText(main_frame, "Ball", Point(10, 90), FONT_HERSHEY_SIMPLEX, 1, CV_RED, 2);
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
#endif