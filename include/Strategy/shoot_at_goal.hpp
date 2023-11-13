/*
 * AIF_ShootAtGoal.h
 *
 *  Created on: Jan 27, 2019
 *      Author: kmeessen
 */

#ifndef AIF_SHOOTATGOAL_H_
#define AIF_SHOOTATGOAL_H_

#include <utils/entity.hpp>
#include <cmath>

#ifndef sqr
#define sqr(a) ((a) * (a))
#endif

#define ACTION_SHOOT_AT_GOAL_MAX_AGE_SEC 0.5
#define N_AIM_ANGLE 100

typedef enum
{
    centerOfGoal = 0, /* put target in the center of the goal */
    randomCorner,     /* pick a random corner in the goal */
    farCorner,        /* pick the far corner (verre hoek) */
    obstacleBased     /* pick the target based on crowdiness */
} ShootAtGoalTargetType_t;

typedef enum ShotType_tag
{
    stFlatShot = 1,
    /* 1 */ /* flat shot at goal */
    stDynamicLob = 2,
    /* 2 */ /* lob shot while driving */
    stStaticLob = 3,
    /* 3 */ /* lob shot during standstill or very low velocity */
    stStaticPass = 4,
    /* 4 */ /* pass during standstill */
    stDynamicPass = 5,
    /* 5 */ /* pass while driving */
    stPenalty = 7,
    /* 7 */                 /* penalty */
    stKinectLob = 8 /* 8 */ /* harder lob when kinect says shot is possible */
} ShotType_t;

typedef struct
{
    ShootAtGoalTargetType_t TargetType;
    ShotType_t ShotType;
    int RotateAroundBall;
} ActionShootAtGoal_t, *pActionShootAtGoal_t;

typedef struct
{
    int State;
} ActionHandlerStateShootAtGoal_t, *pActionHandlerStateShootAtGoal_t;
typedef int ActionReturnValue_t;

ActionReturnValue_t EvaluateActionShootAtGoal(pActionHandlerStateShootAtGoal_t State, double LastTime);

float atan2_approximation(float y, float x)
{
    if (x == 0.0f)
    {
        if (y > 0.0f)
            return M_PI_2;
        if (y == 0.0f)
            return 0.0f;
        return -M_PI_2;
    }
    float atan;
    float z = y / x;
    if (fabs(z) < 1.0f)
    {
        atan = z / (1.0f + 0.28f * z * z);
        if (x < 0.0f)
        {
            if (y < 0.0f)
                return atan - M_PI;
            return atan + M_PI;
        }
    }
    else
    {
        atan = M_PI_2 - z / (z * z + 0.28f);
        if (y < 0.0f)
            return atan - M_PI;
    }
    return atan;
}

/*### distance, angle #########################################################*/
/*-----------------------------------------------------------------------------*/
double getdistance(const double *x, const double *y)
{
    return sqrt(getdistancesquared(x, y));
}

/*-----------------------------------------------------------------------------*/
double getdistancesquared(const double *x, const double *y)
{
    return sqr(x[0] - y[0]) + sqr(x[1] - y[1]);
}

/*-----------------------------------------------------------------------------*/
short getdistance_forshorts(short *p1, short *p2)
{
    return sqrt((p2[0] - p1[0]) * (p2[0] - p1[0]) + (p2[1] - p1[1]) * (p2[1] - p1[1]));
}

/*-----------------------------------------------------------------------------*/
double getdistance_to_origin(const double *p1)
{
    return sqrt(p1[0] * p1[0] + p1[1] * p1[1]);
}

double getangle(const double *x, const double *y)
{
    /* vertical upright axis: phi=0
     * counterclockwise increase of phi
     * x is target point, y is starting point from which angle should be computed
     */

    return atan2_approximation(-(x[0] - y[0]), x[1] - y[1]);
}

double modulus(double x, double y)
{
    x = fmod(x, y);
    if (x < 0)
        x += y;
    return x;
}

static double CheckObstacleCrowdinessInGoalArea(double *angles, int *obstaclefree, const double *sidepoints, const p_RobotEntity obstacles, const PRobotEntity_t pose, const double clearance)
{
    double angle_begin, r, phi, distance[N_AIM_ANGLE];
    int i, j;
    double tmp, a1, a2, p[2], lw, up;
    const double *point1, *point2;

    point1 = sidepoints;
    point2 = sidepoints + 2;

    a1 = modulus(getangle(point1, pose->arr), 2 * M_PI);
    a2 = modulus(getangle(point2, pose->arr), 2 * M_PI);
    if (a2 < a1)
        a2 += 2 * M_PI;
    if (a2 - a1 > M_PI)
    {
        point1 = sidepoints + 2;
        point2 = sidepoints;
    }

    /*compute angles distances of sidepoints*/
    angle_begin = modulus(getangle(point1, pose->arr), 2 * M_PI);

    /*compute angles and distances to grid between sidepoints*/
    for (i = 0; i < N_AIM_ANGLE; ++i)
    {
        p[0] = (point1[0] * (N_AIM_ANGLE - 1 - i) + point2[0] * i) / (N_AIM_ANGLE - 1);
        p[1] = (point1[1] * (N_AIM_ANGLE - 1 - i) + point2[1] * i) / (N_AIM_ANGLE - 1);
        distance[i] = getdistance(p, pose->arr);
        angles[i] = modulus(getangle(p, pose->arr) - angle_begin, 2 * M_PI) + angle_begin;
        obstaclefree[i] = 1;
    }
}
#endif /* AIF_SHOOTATGOAL_H_ */
