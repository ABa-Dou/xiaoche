/**
 ****************************************************************************************************
 * @file        pid.h
 * @brief       PID 闭环控制器 (FF + PI + D, 带 dt)
 *
 * 特性:
 *   - 位置式 PID
 *   - 输入/输出统一为物理单位 (mm/s → PWM units)
 *   - dt 参数化, 积分/微分正确缩放
 *   - 前馈 (FF): 根据目标速度预估 PWM
 *   - 抗积分饱和 (条件积分 + 输出钳位回退)
 *   - 积分限幅
 *   - 输出限幅
 *   - 微分项滤波
 *   - 积分分离
 *   - 暴露 P/I/D/FF 各分量用于调试
 ****************************************************************************************************
 */

#ifndef __PID_H
#define __PID_H

#include "sys.h"

typedef struct {
    float kp;          /* 比例增益: PWM_units / (mm/s) */
    float ki;          /* 积分增益: PWM_units / mm    */
    float kd;          /* 微分增益: PWM_units / (mm/s?) */
    float ff_gain;     /* 前馈增益: PWM_units / (mm/s) */

    float setpoint;    /* 目标值 (mm/s) */
    float feedback;    /* 反馈值 (mm/s) */

    float error;       /* 当前误差 (mm/s) */
    float last_error;  /* 上一拍误差 (mm/s) */
    float integral;    /* 积分累加值 (mm) */
    float derivative;  /* 微分值 (mm/s?) */

    float integral_limit;          /* 积分限幅 (mm) */
    float output_limit;            /* 输出限幅 (PWM units) */
    float integral_separation_err; /* 积分分离阈值 (mm/s) */

    //用于打印输出调试
    float p_out;       /* P 分量 (PWM units) */
    float i_out;       /* I 分量 (PWM units) */
    float d_out;       /* D 分量 (PWM units) */
    float ff_out;      /* FF 分量 (PWM units) */
    float output;      /* 最终输出 (PWM units) */
} pid_t;

void  pid_init(pid_t *pid, float kp, float ki, float kd, float ff_gain,
               float integral_limit, float output_limit,
               float integral_separation_err);
float pid_calculate(pid_t *pid, float setpoint, float feedback, float dt);
void  pid_reset(pid_t *pid);
void  pid_set_params(pid_t *pid, float kp, float ki, float kd, float ff_gain);

#endif