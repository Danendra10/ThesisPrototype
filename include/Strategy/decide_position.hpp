#include <utils/entity.hpp>
#include <utils/utils.hpp>
#include <math/vec_attr.hpp>
#include <math/vec_repl.hpp>

#define X_FIELD 800
#define Y_FIELD 1200
#define X_FIELD_1_2 (X_FIELD * 0.5)
#define Y_FIELD_1_2 (Y_FIELD * 0.5)
#define X_FIELD_1_4 (X_FIELD * 0.25)
#define Y_FIELD_1_4 (Y_FIELD * 0.25)

/**
 * @return the position to go
 */
static float *PositionToGo(const PRobotEntity_t robot, const PRobotEntity_t obstacles, const BallEntity ball, uint8_t step_size, Attractive::Force attractive_force, Repulsive::Force repuslive_force, uint16_t flags = approach_ball)
{
    float *position = new float[2];
    position[0] = 1;
    position[1] = 1;

    float *target = new float[2];
    target[0] = (flags & approach_ball == approach_ball) ? ball->pose_x : 800;
    target[1] = (flags & approach_ball == approach_ball) ? ball->pose_y : 1200;

    float least_force_x = __FLT_MAX__;
    float least_force_y = __FLT_MAX__;
    float least_force_x_idx = __FLT_MAX__;
    float least_force_y_idx = __FLT_MAX__;

    for (uint16_t x = 0; x < X_FIELD; x += step_size)
    {
        for (uint16_t y = 0; y < Y_FIELD; y += step_size)
        {
            if (robot->pose_x <= X_FIELD_1_2 && x <= robot->pose_x)
                continue;
            if (robot->pose_x <= X_FIELD_1_4 && x <= X_FIELD_1_4)
                continue;

            float f_x_attr, f_y_attr;
            attractive_force.Update(x, y, target[0], target[1], 20, f_x_attr, f_y_attr);

            float f_x_rep_total = 0, f_y_rep_total = 0;

            uint8_t is_in_obstacle_zone = 0;
            for (int i = 0; i < 2; i++)
            {
                if (sqrt((x - obstacles[i].pose_x) * (x - obstacles[i].pose_x) + (y - obstacles[i].pose_y) * (y - obstacles[i].pose_y)) < 100 ||
                    CheckLineCircleIntersection2(x, y, robot->pose_x, robot->pose_x, obstacles[i].pose_x, obstacles[i].pose_y, 100))
                {
                    is_in_obstacle_zone = 1;
                    break;
                }
            }
        }
    }
}