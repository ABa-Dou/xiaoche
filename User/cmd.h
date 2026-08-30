#ifndef __CMD_H
#define __CMD_H

#include "sys.h"
#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float ff_gain;
    float tgt_speed_mm_s;
    float integral_limit;
    float integral_separation_err;
    float filter_alpha;
    bool  has_kp;
    bool  has_ki;
    bool  has_kd;
    bool  has_ff;
    bool  has_tgt;
    bool  has_ilim;
    bool  has_sep;
    bool  has_flt;
    uint8_t motor_id;  /* 0=all, 1=RF 2=LF 3=RR 4=LR */
} cmd_params_t;

bool cmd_parse(const uint8_t *buf, uint16_t len, cmd_params_t *params);

#endif