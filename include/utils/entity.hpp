#ifndef ENTITY_HPP_
#define ENTITY_HPP_

#include <cstdint>

enum PositioningFlag
{
    approach_ball = 0b00000000,
    approach_ball_preparation = 0b00000001,
    approach_goal = 0b00000010,
    avoid_ball = 0b00000100,
    avoid_goal = 0b00001000,
};

enum AttackerAlgorithm
{
    approach_ball_att = 0b00000000,
    approach_goal_att = 0b00000001,
    avoid_ball_att = 0b00000010,
    avoid_goal_att = 0b00000100,
    pass_ball_att = 0b00001000,
};

typedef struct RobotEntity
{
    double arr[3];
    uint8_t role;
    float pose_x;
    float pose_y;
    float pose_th;
    float last_pose_x;
    float last_pose_y;
    float last_pose_th;
    float vel_x;
    float vel_y;
    float vel_th;
    uint8_t is_passing;
    uint8_t is_positioning;
    uint8_t is_able_to_move;
    uint8_t is_kicking;
    uint8_t is_holding_ball;
    uint8_t is_friend;

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
        uint8_t is_friend_ = 1,
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
                               is_kicking(kicking),
                               is_friend(is_friend_)
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
} RobotEntity_t, *p_RobotEntity;

typedef RobotEntity_t *p_RobotEntity_t;

typedef p_RobotEntity_t PRobotEntity_t;

typedef struct BallEntity
{
    float pose_x;
    float pose_y;

    BallEntity(float x, float y) : pose_x(x), pose_y(y)
    {
    }

    BallEntity() : pose_x(0.0f), pose_y(0.0f)
    {
    }
} BallEntity_t, *p_BallEntity;

typedef BallEntity_t *p_BallEntity_t;

typedef p_BallEntity_t PBallEntity_t;

/**
 * @struct      Point2D_t
 * @brief       Point in 2d space represented by x- and y-coordinate.
 */
typedef struct
{
    double x; /**< x-coordinate of Point2D_t */
    double y; /**< y-coordinate of Point2D_t */
} Point2D_t;

/**
 * @struct      pPoint2D_t
 * @brief       Pointer to a Point2D_t.
 */
typedef Point2D_t *pPoint2D_t;

/**
 * @union      Vec_t
 * @brief       Vector in 3d space.
 */
typedef union
{
    double arr[3];
    struct
    {
        double x; /**< x-coordinate of Vec_t */
        double y; /**< y-coordinate of Vec_t */
        double z; /**< z-coordinate of Vec_t */
    };
    struct
    {
        double filler2[2]; /**< x-coordinate of Vec_t */
        double o;          /**< orientation of Vec_t */
    };
    struct
    {
        Point2D_t xy; /**< x- and y-coordinate of Vec_t, represented by a Point in 2d space; Point2D_t */
        double r;     /**< radius or z-coordinate of Vec_t */
    };
} Vec_t;

/**
 * @struct      Pos_t
 * @brief       Position represented by a vector in 3d space.
 */
typedef Vec_t Pos_t;

#endif