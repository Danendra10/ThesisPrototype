#ifndef PID_H_
#define PID_H_

#include "sys/time.h"

#ifdef _cplusplus
extern "C"
{
#endif

    typedef struct PID_tag
    {
        float Kp;
        float Ki;
        float Kd;
        float min_out;
        float max_out;
        float min_integral;
        float max_integral;
        float integral;
        float last_error;
        float proportional;
        float derivative;
        float output_speed;
        std::chrono::system_clock::time_point last_call;
        uint64_t last_call_ms;
    } PID_t;

    void PIDInit(PID_t *pid, float Kp, float Ki, float Kd)
    {
        pid->Kp = Kp;
        pid->Ki = Ki;
        pid->Kd = Kd;
    }
    float PIDCalculate(PID_t *pid, float error, float minmax)
    {
        // std::chrono::high_resolution_clock::time_point t_now = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> elapsed_seconds = t_now - pid->last_call;
        // if (elapsed_seconds.count() > 2)
        // {
        //     pid->integral = 0;
        //     pid->last_error = 0;
        // }
        // pid->last_call = std::chrono::high_resolution_clock::now();

        timeval tim;
        gettimeofday(&tim, NULL);
        uint64_t time_now_ms = 1.0e3 * tim.tv_sec + tim.tv_usec * 1.0e-3;

        // if(time_now_ms - pid->last_call_ms > 2000){
        //     pid->integral = 0;
        //     pid->last_error = 0;
        //     printf("RESET CAKK\n");
        // }
        pid->last_call_ms = time_now_ms;

        pid->min_out = pid->min_integral = -minmax;
        pid->max_out = pid->max_integral = minmax;

        pid->proportional = pid->Kp * error;
        pid->integral += pid->Ki * error;
        pid->derivative = pid->Kd * (error - pid->last_error);

        // printf("%.2f %.2f %.2f %.2f %.2f\n", pid->proportional, pid->integral, pid->derivative, error, pid->last_error);
        pid->last_error = error;

        if (pid->integral > pid->max_integral)
            pid->integral = pid->max_integral;
        else if (pid->integral < pid->min_integral)
            pid->integral = pid->min_integral;

        pid->output_speed = pid->proportional + pid->integral + pid->derivative;

        if (pid->output_speed > pid->max_out)
            pid->output_speed = pid->max_out;
        else if (pid->output_speed < pid->min_out)
            pid->output_speed = pid->min_out;
        return pid->output_speed;
    }

    void PIDReset(PID_t *pid)
    {
        pid->integral = 0;
    }

#ifdef _cplusplus
}
#endif

#endif // PID_H_