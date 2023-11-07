#include <math.h>

namespace Attractive
{
    class Force
    {
    private:
        float constanta = 1.0;
        float constanta_a = 1.0;
        float constanta_b = -1.0;
        float threshold_distance = 75;

    public:
        Force();
        ~Force();

        void Init(float k_attr, float curvature_a, float curvature_b, float threshold_distance_);
        void Update(float x_curr, float y_curr, float x_goal, float y_goal, float max_vel, float &f_x, float &f_y);
        float GetConstanta();
        float GetConstantaA();
        float GetConstantaB();
    };

    //------------------------------------------------------------------------------
    //==============================================================================

    Force::Force()
    {
    }

    Force::~Force()
    {
    }

    //------------------------------------------------------------------------------
    //==============================================================================

    void Force::Init(float k_attr, float curvature_a, float curvature_b, float threshold_distance_)
    {
        constanta = k_attr;
        constanta_a = curvature_a;
        constanta_b = curvature_b;
        threshold_distance = threshold_distance_;
    }

    void Force::Update(float x_curr, float y_curr, float x_goal, float y_goal, float max_vel, float &f_x, float &f_y)
    {
        float dist_to_goal = sqrt((x_curr - x_goal) * (x_curr - x_goal) + (y_curr - y_goal) * (y_curr - y_goal));

        if (dist_to_goal < threshold_distance)
        {
            f_x = -constanta * (x_curr - x_goal) / (constanta_a * constanta_a);
            f_y = -constanta * (y_curr - y_goal) / (constanta_b * constanta_b);
        }
        else
        {
            float grad_x = (x_curr == x_goal) ? 0 : (x_curr - x_goal) / fabs(x_curr - x_goal);
            float grad_y = (y_curr == y_goal) ? 0 : (y_curr - y_goal) / fabs(y_curr - y_goal);

            f_x = -constanta * grad_x * max_vel / (constanta_a * constanta_a);
            f_y = -constanta * grad_y * max_vel / (constanta_b * constanta_b);
        }
    }

    float Force::GetConstanta()
    {
        return constanta;
    }
    float Force::GetConstantaA()
    {
        return constanta_a;
    }
    float Force::GetConstantaB()
    {
        return constanta_b;
    }
}