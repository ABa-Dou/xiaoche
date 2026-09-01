/**
 ****************************************************************************************************
 * @file        motor.h
 * @brief       电机驱动代码(PWM + 方向 + 编码器)
 *
 * 电机型号: L520 永磁有刷直流电机
 * 额定电压: 12V
 * 减速比:   1:40
 * 输出轴转速: 300rpm(±5%)
 * 编码器:   AB相增量式霍尔, 11线, 自带上拉整形
 * 编码器分辨率: 11线 × 4倍频 × 40减速比 = 1760 脉冲/输出轴转
 ****************************************************************************************************
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"
#include "easy_log.h"
typedef enum {
    MOTOR_LF = 0,
    MOTOR_RF,
    MOTOR_LR,
    MOTOR_RR,
    MOTOR_COUNT
} motor_id_t;

#define MOTOR_SPEED_MAX         1000

#define ENCODER_LINES           11
#define ENCODER_MODE_X4         4
#define GEAR_RATIO              40
#define ENCODER_PULSES_PER_REV  (ENCODER_LINES * ENCODER_MODE_X4 * GEAR_RATIO)

void motor_init(void);
void motor_set_speed(motor_id_t id, int16_t speed);
void motor_encoder_update(void);
int32_t motor_get_encoder(motor_id_t id);
void motor_reset_encoder(motor_id_t id);
int32_t motor_get_output_rev(motor_id_t id);
int32_t motor_get_output_angle(motor_id_t id);

#endif