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

bool CheckLineCircleIntersection2(float x1, float y1, float x2, float y2, float cx, float cy, float r)
{
    float dist = std::abs((y2 - y1) * cx - (x2 - x1) * cy + x2 * y1 - y2 * x1) /
                 std::sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));

    if (dist <= r)
    {
        return true;
    }

    return false;
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

#endif