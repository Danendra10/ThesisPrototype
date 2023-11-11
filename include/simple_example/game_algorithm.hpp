#ifndef GAME_ALGORITHM_HPP_
#define GAME_ALGORITHM_HPP_

#define X_FIELD 1300
#define Y_FIELD 900
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
#include <simple_example/machine_state.hpp>
#include <utils/track_keyboard.hpp>
#include <utils/timer.hpp>
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

// Global queue and mutex for UI tasks
std::queue<std::function<void()>> ui_tasks;
std::mutex ui_tasks_mutex;
std::condition_variable ui_tasks_cv;
std::atomic<bool> finished{false};

logger::Logger logger_instance;

MachineState main_state;
MachineState sub_state;
RobotEntity robot[2] = {
    RobotEntity(1, 500, 100, 90),
    RobotEntity(1, 1000, 700, 90)};

BallEntity ball = {
    X_FIELD_1_2, Y_FIELD_1_2};

Point2f goal_pose(X_FIELD, Y_FIELD_1_2);

extern uint8_t game_status;
extern uint8_t robot_base_action;

Mat main_frame = Mat(Y_FIELD, X_FIELD, CV_8UC3, Scalar(0, 0, 0));

int TOTAL_ = 400;
int stepSize = 17;
int maxVel = 2;
int Kattr = 1;
int Krepl = 89381793;
int d0 = 400;
int a = 1;
int b = 1;
int _speed = 77;

vector<Point2f> obstacles = {
    {50, 50}, {300, 300}, {500, 700}, {700, 700}, {1000, 300}};

Point2f robot_target;
Point2f last_robot_target;
Point2f robot_vel;

uint8_t exclude_radius_around_obstacles = 100;
uint8_t robot_radius = 20;
uint8_t restricted_min_passing_areas_friend = 50;
uint16_t restricted_max_passing_areas_friend = 300;
uint16_t max_robot_movement_radius = 300;

int8_t num_robot_holding_ball = -1;

float min_receiver_rad = robot_radius + restricted_min_passing_areas_friend;
float max_receiver_rad = robot_radius + restricted_max_passing_areas_friend;

Attractive::Force attractive_force;
Repulsive::Force repulsive_force;

Mat f_attr_x(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));
Mat f_attr_y(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));
Mat f_rep_x(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));
Mat f_rep_y(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));
Mat f_total_x(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));
Mat f_total_y(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));

int Initialize();
void MainGameCallback();
void DrawField();
void DrawOurRobot();
void DrawBall();
void ClearFrame();
void Display();
void CountRobotTarget(Point2f &point);
void CountDesiredVel();
void MainAlogirthm();
void UpdateRobotLastPose(uint8_t num);
uint8_t RobotCanShoot(uint8_t num);

void processUITasks();
void enqueueUITask(std::function<void()> task);
#endif