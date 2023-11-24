#include <math/trigonom.hpp>

#define FastSqr(x) ((x) * (x))

float FastInvSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int *)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float *)&i;
    x = x * (1.5f - xhalf * x * x);
    return x;
}
// #define Distance(x1, y1, x2, y2) FastInvSqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
