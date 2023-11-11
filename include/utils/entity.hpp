#ifndef ENTITY_HPP_
#define ENTITY_HPP_

#include <cstdint>

struct RobotEntity
{
    uint8_t role;
    float pose_x;
    float pose_y;
    float pose_th;
    float last_pose_x;
    float last_pose_y;
    float last_pose_th;
    uint8_t is_passing;
    uint8_t is_positioning;
    uint8_t is_able_to_move;
    uint8_t is_kicking;
    uint8_t is_holding_ball;

    /**
     * @param role
     * @param x
     * @param y
     * @param th
     * @param passing = 0
     * @param positioning = 0
     * @param able_to_move = 0
     * @param kicking = 0
     */
    RobotEntity(
        uint8_t role_,
        float x,
        float y,
        float th,
        uint8_t passing = 0,
        uint8_t positioning = 0,
        uint8_t able_to_move = 0,
        uint8_t kicking = 0) : role(role_),
                               pose_x(x),
                               pose_y(y),
                               pose_th(th),
                               last_pose_x(x),
                               last_pose_y(y),
                               last_pose_th(th),
                               is_passing(passing),
                               is_positioning(positioning),
                               is_able_to_move(able_to_move),
                               is_kicking(kicking)
    {
    }

    // Default constructor
    RobotEntity() : role(0),
                    pose_x(0.0f),
                    pose_y(0.0f),
                    pose_th(0.0f),
                    last_pose_x(0.0f),
                    last_pose_y(0.0f),
                    last_pose_th(0.0f),
                    is_passing(0.0f),
                    is_positioning(0.0f),
                    is_able_to_move(0.0f),
                    is_kicking(0.0f)
    {
    }
};

struct BallEntity
{
    float pose_x;
    float pose_y;

    BallEntity(float x, float y) : pose_x(x), pose_y(y)
    {
    }

    BallEntity() : pose_x(0.0f), pose_y(0.0f)
    {
    }
};

#endif