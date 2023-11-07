#include <math.h>

namespace Repulsive
{
    class Force
    {
    private:
        float constanta = 1.0;
        float threshold_distance = 75;

    public:
        Force();
        ~Force();

        void Init(float k_repl, float threshold_distance_);
        void Update(float x_curr, float y_curr, float x_obstacle, float y_obstacle, float &f_x, float &f_y);
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

    void Force::Init(float k_repl, float threshold_distance_)
    {
        constanta = k_repl;
        threshold_distance = threshold_distance_;
    }

    void Force::Update(float x_curr, float y_curr, float x_obstacle, float y_obstacle, float &f_x, float &f_y)
    {
        // Vector from obstacle to current position
        float vec_x = x_curr - x_obstacle;
        float vec_y = y_curr - y_obstacle;

        // Calculate the distance from the obstacle to the current position
        float d = sqrt(vec_x * vec_x + vec_y * vec_y);
        f_x = -(constanta * (1 / d - 1 / threshold_distance) * (1 / (d * d)) * (vec_x / d)) * (d <= threshold_distance && d != 0);
        f_y = -(constanta * (1 / d - 1 / threshold_distance) * (1 / (d * d)) * (vec_y / d)) * (d <= threshold_distance && d != 0);
    }

    float Force::GetConstanta()
    {
        return constanta;
    }
}