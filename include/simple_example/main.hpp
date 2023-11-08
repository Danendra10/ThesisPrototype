#ifndef SIMPLE_EXAMPLE_MAIN_HPP
#define SIMPLE_EXAMPLE_MAIN_HPP

#define X_FIELD 1200
#define Y_FIELD 800

#include <math/vec_attr.hpp>
#include <math/vec_repl.hpp>
#include <utils/utils.hpp>
#include <logger/logger.hpp>

#include <opencv2/opencv.hpp>

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
int Krepl = 10000000;
int d0 = 50;
int a = 1;
int b = 1;
int _speed = 10;

int exclude_radius_around_obstacles = 100;
int robot_radius = 20;
int restricted_min_passing_areas_friend = 50;
int restricted_max_passing_areas_friend = 300;

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

#endif