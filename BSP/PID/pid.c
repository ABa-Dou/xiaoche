/**
 ****************************************************************************************************
 * @file        pid.c
 * @brief       PID ????????? (FF + PI + D, ?? dt)
 *
 * ???:
 *   output = Kp * e  +  Ki * ??e dt  +  Kd * de/dt  +  FF_gain * setpoint
 *
 * ?????????:
 *   ????¦Ë??, ?????????????????, ???????????
 * ???????:
 *   ????????????????, ???????
 * ??????:
 *   ????????
 ****************************************************************************************************
 */

#include "pid.h"
#include <math.h>

#define PID_DERIVATIVE_FILTER_ALPHA  0.6f

void pid_init(pid_t *pid, float kp, float ki, float kd, float ff_gain,
              float integral_limit, float output_limit,
              float integral_separation_err)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->ff_gain = ff_gain;

    pid->setpoint = 0.0f;
    pid->feedback = 0.0f;

    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;

    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->integral_separation_err = integral_separation_err;

    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->ff_out = 0.0f;
    pid->output = 0.0f;
}

float pid_calculate(pid_t *pid, float setpoint, float feedback, float dt)
{
    float p_out, i_out, d_out, ff_out;
    float raw_derivative;
    float integral_before;

    if (dt <= 0.0f) dt = 0.05f;

    pid->setpoint = setpoint;


    pid->feedback = feedback;


    pid->error = setpoint - feedback;

  
    p_out = pid->kp * pid->error;

    
    /* integral_before = pid->integral;
    if (pid->integral_separation_err <= 0.0f ||
        fabsf(pid->error) <= pid->integral_separation_err)
    {
        pid->integral += pid->error * dt;
        if (pid->integral_limit > 0.0f)
        {
            if (pid->integral > pid->integral_limit)
                pid->integral = pid->integral_limit;
            else if (pid->integral < -pid->integral_limit)
                pid->integral = -pid->integral_limit;
        }
    }
    i_out = pid->ki * pid->integral; */

    /* D ?? (??????) */
    /* raw_derivative = (pid->error - pid->last_error) / dt;
    pid->derivative = PID_DERIVATIVE_FILTER_ALPHA * raw_derivative
                    + (1.0f - PID_DERIVATIVE_FILTER_ALPHA) * pid->derivative;
    d_out = pid->kd * pid->derivative; */

    /* pid->last_error = pid->error; */


    ff_out = pid->ff_gain * setpoint;

    pid->output = p_out + ff_out;

    if (pid->output_limit > 0.0f)
    {
        if (pid->output > pid->output_limit)
            pid->output = pid->output_limit;
        else if (pid->output < -pid->output_limit)
            pid->output = -pid->output_limit;
    }

    pid->p_out = p_out;
    pid->ff_out = ff_out;

    return pid->output;
}

void pid_reset(pid_t *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->ff_out = 0.0f;
    pid->output = 0.0f;
}

void pid_set_params(pid_t *pid, float kp, float ki, float kd, float ff_gain)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->ff_gain = ff_gain;
}