#ifndef PID_H
#define PID_H

typedef struct {
    float Kp, Ki, Kd;
    float setpoint;
    float dt;
    float prev_error;
    float prev_prev_error;
    float prev_output;
    float out_min, out_max;
    float deadband;
} IncPID_t;

void IncPID_Init(IncPID_t *p, float kp, float ki, float kd,
                 float dt, float out_min, float out_max);
float IncPID_Compute(IncPID_t *p, float measurement);
void IncPID_SetOutput(IncPID_t *p, float output);

#endif
