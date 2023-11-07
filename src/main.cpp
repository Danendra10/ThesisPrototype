#include <simple_example/main.hpp>

int main()
{
    const int TOTAL_ = 400;
    const int stepSize = 32;
    const float maxVel = 100.0f;
    const float Kattr = 1.0f;
    const float Krepl = 10000000.0f;
    const float d0 = 50.0f;
    const float a = 1.0f; // parabolic curvature of the attractive potential
    const float b = 1.0f; // parabolic curvature of the attractive potential

    // Initialize goal
    Point2f goal(TOTAL_ * 0.5, TOTAL_ * 0.5);

    // Define obstacles
    vector<Point2f> obstacles = {
        {50, 50}, {300, 300}, {500, 700}, {700, 700}, {800, 1000}};

    // Initialize forces
    Attractive::Force attractive_force;
    attractive_force.Init(Kattr, a, b, TOTAL_);

    Repulsive::Force repulsive_force;
    repulsive_force.Init(Krepl, d0);

    // Initialize force matrices
    Mat f_attr_x(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));
    Mat f_attr_y(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));
    Mat f_rep_x(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));
    Mat f_rep_y(Size(TOTAL_ + 1, TOTAL_ + 1), CV_32F, Scalar(0));

    for (int i = 0; i <= TOTAL_; i += stepSize)
    {
        for (int j = 0; j <= TOTAL_; j += stepSize)
        {
            // Current position
            float x_curr = i;
            float y_curr = j;

            // Calculate attractive force
            float f_x_attr, f_y_attr;
            attractive_force.Update(x_curr, y_curr, goal.x, goal.y, maxVel, f_x_attr, f_y_attr);
            f_attr_x.at<float>(i, j) = f_x_attr;
            f_attr_y.at<float>(i, j) = f_y_attr;

            // Calculate repulsive force from each obstacle
            float f_x_rep_total = 0, f_y_rep_total = 0;
            for (const auto &obstacle : obstacles)
            {
                float f_x_rep, f_y_rep;
                repulsive_force.Update(x_curr, y_curr, obstacle.x, obstacle.y, f_x_rep, f_y_rep);
                f_x_rep_total += f_x_rep;
                f_y_rep_total += f_y_rep;
            }
            f_rep_x.at<float>(i, j) = f_x_rep_total;
            f_rep_y.at<float>(i, j) = f_y_rep_total;
        }
    }

    while (true)
    {
        visualizeVectorField(f_attr_x + f_rep_x, f_attr_y + f_rep_y, obstacles, goal);
        // Display();
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }
}

void Display()
{
    imshow("Display", main_frame);
}

// Visualization function
void visualizeVectorField(const Mat &field_x, const Mat &field_y, const vector<Point2f> &obstacles, const Point2f &goal)
{
    // Create an image to draw the vector field
    Mat display = Mat::zeros(field_x.size(), CV_8UC3);

    // Normalize the magnitude of vectors for consistent arrow length
    double maxVal;
    minMaxLoc(field_x, nullptr, &maxVal);
    double scaleFactor = 50.0 / maxVal; // Adjust 50.0 to scale the arrows as needed

    // Draw the vector field
    for (int i = 0; i < field_x.rows; i++)
    {
        for (int j = 0; j < field_x.cols; j++)
        {
            Point2f start(j, i);
            // Scale the force to a visible size
            Point2f end = start + Point2f(field_x.at<float>(i, j), field_y.at<float>(i, j)) * scaleFactor;
            // Draw the line for the vector
            arrowedLine(display, start, end, Scalar(255, 0, 0), 1, LINE_AA);
        }
    }

    // Draw obstacles as circles
    for (const auto &obstacle : obstacles)
    {
        circle(display, obstacle, 10, Scalar(0, 0, 255), -1); // -1 for filled circle
    }

    // Draw the goal as a green circle
    circle(display, goal, 10, Scalar(0, 255, 0), -1);

    // Show the image
    imshow("Vector Field", display);
    waitKey(0); // Wait for a key press to exit
}