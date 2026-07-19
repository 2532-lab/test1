#include "PID.h"
#include <math.h>

void IncPID_Init(IncPID_t *p, float kp, float ki, float kd,
                 float dt, float out_min, float out_max)
{
    p->Kp = kp; p->Ki = ki; p->Kd = kd;
    p->dt = dt;
    p->setpoint = 0.0f;
    p->prev_error = 0.0f;
    p->prev_prev_error = 0.0f;
    p->prev_output = 0.0f;
    p->out_min = out_min;
    p->out_max = out_max;
    p->deadband = 0.0f;
}

float IncPID_Compute(IncPID_t *p, float measurement)
{
    float error = p->setpoint - measurement;
    if (fabsf(error) < p->deadband) {
        p->prev_prev_error = p->prev_error;
        p->prev_error = error;
        return p->prev_output;
    }
    float delta = p->Kp * (error - p->prev_error)
                + p->Ki * error * p->dt
                + p->Kd * (error - 2.0f * p->prev_error + p->prev_prev_error) / p->dt;
    p->prev_prev_error = p->prev_error;
    p->prev_error = error;
    float output = p->prev_output + delta;
    if (output > p->out_max) output = p->out_max;
    if (output < p->out_min) output = p->out_min;
    p->prev_output = output;
    return output;
}

void IncPID_SetOutput(IncPID_t *p, float output)
{
    p->prev_output = output;
    p->prev_error = 0.0f;
    p->prev_prev_error = 0.0f;
}
