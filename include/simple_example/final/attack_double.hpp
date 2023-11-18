#ifndef ATTACK_DOUBLE_HPP_
#define ATTACK_DOUBLE_HPP_

#define X_FIELD 1300
#define Y_FIELD 900
#define X_FIELD_1_2 (X_FIELD * 0.5)
#define Y_FIELD_1_2 (Y_FIELD * 0.5)
#define X_FIELD_1_4 (X_FIELD * 0.25)
#define Y_FIELD_1_4 (Y_FIELD * 0.25)
#define X_FIELD_3_4 (X_FIELD * 0.75)
#define Y_FIELD_3_4 (Y_FIELD * 0.75)

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

#include <math/new_vec_attr.hpp>
#include <math/new_vec_repl.hpp>
#include <utils/utils.hpp>
#include <logger/logger.hpp>
#include <simple_example/machine_state.hpp>
#include <utils/entity.hpp>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define CV_RED Scalar(0, 0, 255)
#define CV_GREEN Scalar(0, 255, 0)
#define CV_BLUE Scalar(255, 0, 0)
#define CV_YELLOW Scalar(0, 255, 255)
#define CV_MAGENTA Scalar(255, 0, 255)
#define CV_CYAN Scalar(255, 255, 0)
#define CV_WHITE Scalar(255, 255, 255)
#define CV_BLACK Scalar(0, 0, 0)

logger::Logger logger_instance;

MachineState main_state;
MachineState sub_state;

int8_t num_robot_handling_ball = -1;
int8_t num_enemy_handling_ball = -1;

int x_enemy_1 = X_FIELD_1_2;
int y_enemy_1 = Y_FIELD_1_2;
int x_enemy_2 = X_FIELD_3_4 + 100;
int y_enemy_2 = Y_FIELD_3_4 - 100;

RobotEntity robot[2] = {
    RobotEntity(1, X_FIELD_1_4, Y_FIELD_1_2, 0),
    RobotEntity(1, X_FIELD_1_4, Y_FIELD_3_4, 0)};

RobotEntity enemy[2] = {
    RobotEntity(0, x_enemy_1, y_enemy_1, 90),
    RobotEntity(0, x_enemy_2, y_enemy_2, 90)};

BallEntity ball = {X_FIELD_1_2, Y_FIELD_1_2};

Point2f goal_pose(X_FIELD - 50, Y_FIELD_1_2);

extern uint8_t game_status;
extern uint8_t robot_base_action;

Mat main_frame = Mat(Y_FIELD, X_FIELD, CV_8UC3, Scalar(0, 0, 0));

int stepSize = 17;
int linear_slope = 2;
int Kattr = 1;
int Krepl = 100000000;
int d0 = 400;
int a = 1;
int b = 1;
float c = 0.04;
int rad_thr = 50;
int u_trans = 100;
int _speed = 77;

Point2f attacker_robot_target;
Point2f attacker_movement_points = {robot[0].pose_x, robot[0].pose_y};
vector<Point2f> attacker_vec_movement_points;
Point2f attacker_robot_vel;
Point2f assist_robot_target;
Point2f assist_movement_points = {robot[1].pose_x, robot[1].pose_y};
vector<Point2f> assist_vec_movement_points;
Point2f assist_robot_vel;
Point2f last_robot_target;
uint8_t exclude_radius_around_obstacles = 100;
uint8_t robot_radius = 20;
uint8_t restricted_min_passing_areas_friend = 50;
uint16_t restricted_max_passing_areas_friend = 300;
uint16_t max_robot_movement_radius = 300;

int8_t num_robot_holding_ball = -1;

float min_receiver_rad = robot_radius + restricted_min_passing_areas_friend;
float max_receiver_rad = robot_radius + restricted_max_passing_areas_friend;

Mat f_attr_x(Size(Y_FIELD, X_FIELD), CV_32F, Scalar(0));
Mat f_attr_y(Size(Y_FIELD, X_FIELD), CV_32F, Scalar(0));
Mat f_rep_x(Size(Y_FIELD, X_FIELD), CV_32F, Scalar(0));
Mat f_rep_y(Size(Y_FIELD, X_FIELD), CV_32F, Scalar(0));
Mat f_total_x(Size(Y_FIELD, X_FIELD), CV_32F, Scalar(0));
Mat f_total_y(Size(Y_FIELD, X_FIELD), CV_32F, Scalar(0));

int Initialize();
void MainGameCallback();
void DrawField();
void DrawOurRobot();
void DrawBall();
void ClearFrame();
void Display();
void CountRobotTarget(uint8_t robot_num, float &target_x, float &target_y);
void CountDesiredVel(uint8_t robot_num);
void MainAlgorithm();

void DrawRobot(float x_, float y_, float th_, Scalar color_);

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
        ball.pose_x = (robot[num_robot_handling_ball].pose_x + 35 * cos(robot[num_robot_handling_ball].pose_th * DEG2RAD));
        ball.pose_y = (robot[num_robot_handling_ball].pose_y + 35 * sin(robot[num_robot_handling_ball].pose_th * DEG2RAD));
    }
    else if (num_enemy_handling_ball != -1)
    {
        ball.pose_x = (enemy[num_enemy_handling_ball].pose_x + 35 * cos(enemy[num_enemy_handling_ball].pose_th * DEG2RAD));
        ball.pose_y = (enemy[num_enemy_handling_ball].pose_y + 35 * sin(enemy[num_enemy_handling_ball].pose_th * DEG2RAD));
    }
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
Point2f MidPointTriangle(float x1, float y1, float x2, float y2, float x3, float y3)
{
    Point2f midpoint;
    midpoint.x = (x1 + x2 + x3) / 3.0;
    midpoint.y = (y1 + y2 + y3) / 3.0;
    return midpoint;
}

#endif