#include <iostream>
#include <cmath>
#include <vector>

struct Point
{
    double x, y;
};

// Function to find the intersection points
std::vector<Point> findCircleLineIntersections(double x1, double y1, double x2, double y2,
                                               double cx, double cy, double radius)
{
    std::vector<Point> intersections;

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

    // Find discriminant
    double discriminant = b * b - 4 * a * c;

    // If discriminant is negative, there are no real intersections
    if (discriminant < 0)
    {
        return intersections;
    }

    // Calculate the two intersections
    discriminant = sqrt(discriminant);

    // First intersection
    double t1 = (-b + discriminant) / (2 * a);
    Point intersection1 = {x1 + t1 * dx, y1 + t1 * dy};
    intersections.push_back(intersection1);

    // If discriminant is zero, there is only one intersection
    if (discriminant == 0)
    {
        return intersections;
    }

    // Second intersection
    double t2 = (-b - discriminant) / (2 * a);
    Point intersection2 = {x1 + t2 * dx, y1 + t2 * dy};
    intersections.push_back(intersection2);

    return intersections;
}

bool isCircleLineIntersecting(double x1, double y1, double x2, double y2,
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

    // Find discriminant
    double discriminant = b * b - 4 * a * c;

    // If discriminant is negative, there are no real intersections
    return discriminant >= 0;
}

int main()
{
    double x1 = 0, y1 = 0, x2 = 100, y2 = 100, center_x = 50, center_y = 40, radius = 20;
    // Input values for x1, y1, x2, y2, center_x, center_y, and radius

    std::vector<Point> intersections = findCircleLineIntersections(x1, y1, x2, y2, center_x, center_y, radius);
    bool isIntersecting = isCircleLineIntersecting(x1, y1, x2, y2, center_x, center_y, radius);

    std::cout << "Intersecting: " << isIntersecting << std::endl;

    for (const auto &point : intersections)
    {
        std::cout << "Intersection: (" << point.x << ", " << point.y << ")" << std::endl;
    }

    return 0;
}
