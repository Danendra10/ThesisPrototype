#ifndef UTIlS_HPP
#define UTIlS_HPP

#include <iostream>
#include <vector>

using namespace std;

void meshgrid(const double step_size, const double total, std::vector<std::vector<double>> &X, std::vector<std::vector<double>> &Y)
{
    int size = static_cast<int>(total / step_size) + 1; // Calculate the number of points along one dimension

    // Resize the vectors to the required size
    X.resize(size);
    Y.resize(size);
    for (auto &vec : X)
        vec.resize(size);
    for (auto &vec : Y)
        vec.resize(size);

    // Fill the vectors with the meshgrid coordinates
    for (int i = 0; i < size; ++i)
    {
        for (int j = 0; j < size; ++j)
        {
            X[i][j] = j * step_size;
            Y[i][j] = i * step_size;
        }
    }
}

/**
 * Check if a line intersects with a circle.
 *
 * @param x1 The x-coordinate of the first point on the line.
 * @param y1 The y-coordinate of the first point on the line.
 * @param x2 The x-coordinate of the second point on the line.
 * @param y2 The y-coordinate of the second point on the line.
 * @param cx The x-coordinate of the center of the circle.
 * @param cy The y-coordinate of the center of the circle.
 * @param radius The radius of the circle.
 * @return True if the line intersects with the circle, False otherwise.
 */
bool IsCircleLineIntersecting(double x1, double y1, double x2, double y2,
                              double cx, double cy, double radius)
{
    // Convert line to normalized direction vector
    double dx = x2 - x1;
    double dy = y2 - y1;
    double mag = sqrt(dx * dx + dy * dy);
    dx /= mag;
    dy /= mag;

    // Translate circle to origin
    double translated_x1 = x1 - cx;
    double translated_y1 = y1 - cy;

    // Quadratic coefficients
    double a = dx * dx + dy * dy;
    double b = 2 * (dx * translated_x1 + dy * translated_y1);
    double c = translated_x1 * translated_x1 + translated_y1 * translated_y1 - radius * radius;

    // If discriminant is negative, there are no real intersections
    return b * b - 4 * a * c >= 0;
}

bool CheckLineCircleIntersection2(float x1, float y1, float x2, float y2, float cx, float cy, float r)
{
    float dist = std::abs((y2 - y1) * cx - (x2 - x1) * cy + x2 * y1 - y2 * x1) /
                 std::sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));

    return dist <= r;
}

bool CheckLineCircleIntersection(float x1, float y1, float x2, float y2, float cx, float cy, float r)
{
    printf("Line is intersecting circle with center (%f, %f) and radius %f on points (%f, %f) and (%f, %f)\n", cx, cy, r, x1, y1, x2, y2);
    // Calculate the distance from the circle's center to the line
    float lineLength = std::sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
    float dist = std::abs((y2 - y1) * cx - (x2 - x1) * cy + x2 * y1 - y2 * x1) / lineLength;

    // Check if either end of the line segment is inside the circle
    float dx = cx - x1;
    float dy = cy - y1;
    if (dx * dx + dy * dy <= r * r)
        return true;

    dx = cx - x2;
    dy = cy - y2;
    if (dx * dx + dy * dy <= r * r)
        return true;

    // If the line's closest distance to the circle is more than the radius, no intersection
    if (dist > r)
        return false;

    // Check if the closest point on the line is within the line segment
    float dot = ((cx - x1) * (x2 - x1) + (cy - y1) * (y2 - y1)) / (lineLength * lineLength);
    if (dot < 0.0f || dot > 1.0f)
        return false; // the closest point is outside the line segment

    return true;
}

bool IsOutsideRobotMaxRadius(const int current_x, const int current_y, const int robot_x, const int robot_y, const float max_robot_movement_radius)
{
    return sqrt(pow(current_x - robot_x, 2) + pow(current_y - robot_y, 2)) > max_robot_movement_radius;
}

bool IsOutsideFriendThreshold(int x_curr, int y_curr, const int friend_pose_x, const int friend_pose_y, const float min_receiver_rad, const float max_receiver_rad)
{
    return (sqrt(pow((x_curr - friend_pose_x), 2) + pow((y_curr - friend_pose_y), 2)) <= min_receiver_rad ||
            sqrt(pow((x_curr - friend_pose_x), 2) + pow((y_curr - friend_pose_y), 2)) >= max_receiver_rad);
}

#endif