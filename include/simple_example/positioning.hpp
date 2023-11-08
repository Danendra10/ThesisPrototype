#ifndef POSITIONING_HPP_
#define POSITIONING_HPP_

#define X_FIELD 1200
#define Y_FIELD 800
#define X_FIELD_1_2 (X_FIELD * 0.5)
#define Y_FIELD_1_2 (Y_FIELD * 0.5)
#define X_FIELD_1_4 (X_FIELD * 0.25)
#define Y_FIELD_1_4 (Y_FIELD * 0.25)
#define X_FIELD_3_4 (X_FIELD * 0.75)
#define Y_FIELD_3_4 (Y_FIELD * 0.75)

#include <math/vec_attr.hpp>
#include <math/vec_repl.hpp>
#include <utils/utils.hpp>
#include <logger/logger.hpp>

#include <opencv2/opencv.hpp>

#include <chrono>

#define CV_RED Scalar(0, 0, 255)
#define CV_GREEN Scalar(0, 255, 0)
#define CV_BLUE Scalar(255, 0, 0)
#define CV_YELLOW Scalar(0, 255, 255)
#define CV_MAGENTA Scalar(255, 0, 255)
#define CV_CYAN Scalar(255, 255, 0)
#define CV_WHITE Scalar(255, 255, 255)
#define CV_BLACK Scalar(0, 0, 0)

#include <iostream>

using namespace std;
using namespace cv;

logger::Logger logger_instance;

Mat main_frame = Mat(Y_FIELD, X_FIELD, CV_8UC3, Scalar(0, 0, 0));

int TOTAL_ = 400;
int stepSize = 17;
int maxVel = 2;
int Kattr = 1;
int Krepl = 89381793;
int d0 = 400;
int a = 1;
int b = 1;
int _speed = 10;

uint8_t exclude_radius_around_obstacles = 100;
uint8_t robot_radius = 20;
uint8_t restricted_min_passing_areas_friend = 50;
uint16_t restricted_max_passing_areas_friend = 300;
uint16_t max_robot_movement_radius = 300;

float min_receiver_rad = robot_radius + restricted_min_passing_areas_friend;
float max_receiver_rad = robot_radius + restricted_max_passing_areas_friend;

// Initialize goal
Point2f goal(X_FIELD * 0.5, Y_FIELD * 0.5);

// Define obstacles
vector<Point2f> obstacles = {
    {50, 50}, {300, 300}, {500, 700}, {700, 700}, {800, 1000}};

Point2f ball(X_FIELD * 0.5, Y_FIELD * 0.5);
Point2f robot_pose(100, 500);
Point2f friend_pose(1000, 700);

// Initialize forces
Attractive::Force attractive_force;

Repulsive::Force repulsive_force;

// Initialize force matrices
Mat f_attr_x(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));
Mat f_attr_y(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));
Mat f_rep_x(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));
Mat f_rep_y(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));
Mat f_total_x(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));
Mat f_total_y(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));

void Display();
void visualizeVectorField(const Mat &field_x, const Mat &field_y, const vector<Point2f> &obstacles, const Point2f &goal);
void DrawTriangle(Mat &frame, const float center_x, const float center_y, const float radius, const Scalar color);
void OnUpdate(int, void *);
bool IsOutsideRobotMaxRadius(const int current_x, const int current_y)
{
    return sqrt(pow(current_x - robot_pose.x, 2) + pow(current_y - robot_pose.y, 2)) > max_robot_movement_radius;
}

bool IsOutsideFriendThreshold(int x_curr, int y_curr)
{
    return (sqrt(pow((x_curr - friend_pose.x), 2) + pow((y_curr - friend_pose.y), 2)) <= min_receiver_rad ||
            sqrt(pow((x_curr - friend_pose.x), 2) + pow((y_curr - friend_pose.y), 2)) >= max_receiver_rad);
}

#endif