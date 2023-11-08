#include <simple_example/main.hpp>

int main()
{
    namedWindow("Parameters", WINDOW_AUTOSIZE);
    createTrackbar("Step Size", "Parameters", &stepSize, 100, OnUpdate);
    createTrackbar("Max Velocity", "Parameters", &maxVel, 200, OnUpdate);
    createTrackbar("Kattr", "Parameters", &Kattr, 1000, OnUpdate);
    createTrackbar("Krepl", "Parameters", &Krepl, 100000000, OnUpdate);
    createTrackbar("Repulsive Radius", "Parameters", &d0, 400, OnUpdate);
    createTrackbar("a", "Parameters", &a, 10, OnUpdate);
    createTrackbar("b", "Parameters", &b, 10, OnUpdate);
    createTrackbar("Ball speed", "Parameters", &ball_speed, 100, OnUpdate);

    attractive_force.Init(Kattr, a, b, 300);
    repulsive_force.Init(Krepl, d0);

    while (true)
    {
        // visualizeVectorField(f_attr_x, f_attr_y, obstacles, goal);
        int key = cv::waitKey(1);
        if (key == 'q')
        {
            break;
        }
        else if (key == 'w') // Move up
        {
            ball.y = fmaxf(0, ball.y - ball_speed);
        }
        else if (key == 's') // Move down
        {
            ball.y = fminf(Y_FIELD, ball.y + ball_speed);
        }
        else if (key == 'a') // Move left
        {
            ball.x = fmaxf(0, ball.x - ball_speed);
        }
        else if (key == 'd') // Move right
        {
            logger_instance.Log(logger::GREEN, "Moving right");
            ball.x = fminf(X_FIELD, ball.x + ball_speed);
        }

        // After moving the ball, redraw the scene with updated ball position
        main_frame = Scalar::all(0); // Clear the frame or set to your background

        // Draw the obstacles and ball again with updated position
        for (const auto &obstacle : obstacles)
        {
            circle(main_frame, obstacle, 10, Scalar(0, 0, 255), -1); // Draw obstacles
        }
        circle(main_frame, ball, 10, Scalar(255, 255, 0), -1); // Draw ball with updated position

        for (int i = 0; i <= X_FIELD; i += stepSize)
        {
            for (int j = 0; j <= Y_FIELD; j += stepSize)
            {
                // Current position
                float x_curr = i;
                float y_curr = j;

                // Calculate attractive force
                float f_x_attr, f_y_attr;
                attractive_force.Update(x_curr, y_curr, ball.x, ball.y, maxVel, f_x_attr, f_y_attr);
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

                // Total force
                float f_x_total = f_attr_x.at<float>(i, j) + f_rep_x.at<float>(i, j);
                float f_y_total = f_attr_y.at<float>(i, j) + f_rep_y.at<float>(i, j);

                f_total_x.at<float>(i, j) = f_x_total;
                f_total_y.at<float>(i, j) = f_y_total;

                line(main_frame, Point(i, j), Point(i + f_x_total, j + f_y_total), Scalar(0, 255, 0), 1);
            }
        }

        Display();
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    destroyAllWindows();
    return 0;
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
}

void OnUpdate(int, void *)
{
    // clear the frame
    main_frame = Mat(Y_FIELD, X_FIELD, CV_8UC3, Scalar(0, 0, 0));
    attractive_force.Init(Kattr, a, b, 300);
    repulsive_force.Init(Krepl, d0);
}