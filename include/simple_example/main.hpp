#ifndef SIMPLE_EXAMPLE_MAIN_HPP
#define SIMPLE_EXAMPLE_MAIN_HPP

#define X_FIELD 1200
#define Y_FIELD 800

#include <math/vec_attr.hpp>
#include <math/vec_repl.hpp>
#include <utils/utils.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>

using namespace std;
using namespace cv;

Mat main_frame = Mat(Y_FIELD, X_FIELD, CV_8UC3, Scalar(0, 0, 0));
void Display();
void visualizeVectorField(const Mat &field_x, const Mat &field_y, const vector<Point2f> &obstacles, const Point2f &goal);

#endif