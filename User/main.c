#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "easy_log.h"
#include "motor.h"
#include "pid.h"
#include "cmd.h"

#define WHEEL_DIAMETER_MM       65.0f
#define WHEEL_CIRCUMFERENCE_MM  (WHEEL_DIAMETER_MM * 3.14159265f)
#define PID_PERIOD_MS           5
#define PID_PERIOD_S            (PID_PERIOD_MS / 1000.0f)

/* 默认值 (mm/s 域) */
#define DEFAULT_KP              1.0f
#define DEFAULT_KI              4.0f
#define DEFAULT_KD              0.0f
#define DEFAULT_FF_GAIN         1.1f
#define DEFAULT_TGT_SPEED       150.0f
#define DEFAULT_INTEGRAL_LIMIT  50.0f
#define DEFAULT_INTEGRAL_SEP    50.0f
#define DEFAULT_FILTER_ALPHA    0.30f

/* 用户电机编号(1=RF 2=LF 3=RR 4=LR) → motor_id_t */
static const motor_id_t s_motor_map[5] = {
    0,             /* [0] unused */
    MOTOR_RF,      /* [1] RF */
    MOTOR_LF,      /* [2] LF */
    MOTOR_RR,      /* [3] RR */
    MOTOR_LR       /* [4] LR */
};

//一个脉冲对应的距离mm
static float pulses_per_mm(void)
{
    return (float)ENCODER_PULSES_PER_REV / WHEEL_CIRCUMFERENCE_MM;
}

static float pulses_per_period_to_speed(float pulses)
{
    return pulses / pulses_per_mm() / PID_PERIOD_S;
}

void apply_params(pid_t pid[], float *filter_alpha,
                         cmd_params_t *cmd, float tgt_speed_mm_s[])
{
    int start = 0, end = MOTOR_COUNT;
    if (cmd->motor_id > 0) {
        start = (int)s_motor_map[cmd->motor_id];
        end = start + 1;
    }

    for (int i = start; i < end; i++) {
        if (cmd->has_kp)   pid[i].kp = cmd->kp;
        if (cmd->has_ki)   pid[i].ki = cmd->ki;
        if (cmd->has_kd)   pid[i].kd = cmd->kd;
        if (cmd->has_ff)   pid[i].ff_gain = cmd->ff_gain;
        if (cmd->has_ilim) pid[i].integral_limit = cmd->integral_limit;
        if (cmd->has_sep)  pid[i].integral_separation_err = cmd->integral_separation_err;
        if (cmd->has_tgt)  tgt_speed_mm_s[i] = cmd->tgt_speed_mm_s;
    }
    if (cmd->has_flt)  *filter_alpha = cmd->filter_alpha;

    if (cmd->motor_id == 0) {
        LOGI("Params[ALL]: P=%.2f I=%.2f D=%.2f FF=%.2f tgt=%.1fmm/s ilim=%.0f sep=%.1f flt=%.2f",
             pid[0].kp, pid[0].ki, pid[0].kd, pid[0].ff_gain, tgt_speed_mm_s[0],
             pid[0].integral_limit, pid[0].integral_separation_err, *filter_alpha);
    } else {
        LOGI("Params[%d]: P=%.2f I=%.2f D=%.2f FF=%.2f tgt=%.1fmm/s ilim=%.0f sep=%.1f flt=%.2f",
             cmd->motor_id,
             pid[start].kp, pid[start].ki, pid[start].kd, pid[start].ff_gain,
             tgt_speed_mm_s[start],
             pid[start].integral_limit, pid[start].integral_separation_err, *filter_alpha);
    }
}

int main(void)
{
    pid_t pid[MOTOR_COUNT];
    cmd_params_t cmd;
    int32_t last_enc[MOTOR_COUNT] = {0};
    int32_t cur_enc = 0;
    float speed_pulse = 0.0f;
    float speed_raw_mm_s = 0.0f;
    float speed_filtered[MOTOR_COUNT] = {0.0f};
    float tgt_speed_mm_s[MOTOR_COUNT];
    float filter_alpha = DEFAULT_FILTER_ALPHA;
    float pid_out = 0.0f;
    int16_t pwm_value = 0;
    float pwm_percent = 0.0f;
    uint32_t last_tick = 0;

    HAL_Init();
    sys_stm32_clock_init(336, 8, 2, 7);
    delay_init(168);
    usart_init(115200);
    led_init();
    motor_init();

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_reset_encoder((motor_id_t)i);
        pid_init(&pid[i], DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_FF_GAIN,
                 DEFAULT_INTEGRAL_LIMIT, (float)MOTOR_SPEED_MAX, DEFAULT_INTEGRAL_SEP);
        tgt_speed_mm_s[i] = DEFAULT_TGT_SPEED;
    }

    LOGW("Init OK. Cmd: pid[1-4] kp=X ki=X kd=X ff=X tgt=X ilim=X sep=X flt=X");
    LOGW("Default: P=%.2f I=%.2f D=%.2f FF=%.2f tgt=%.1fmm/s ilim=%.0f sep=%.1f flt=%.2f",
         DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_FF_GAIN, DEFAULT_TGT_SPEED,
         DEFAULT_INTEGRAL_LIMIT, DEFAULT_INTEGRAL_SEP, filter_alpha);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        last_enc[i] = motor_get_encoder((motor_id_t)i);
    }
    last_tick = HAL_GetTick();

    while (1)
    {
        if (g_usart_rx_sta & 0x8000)
        {
            uint16_t rx_len = g_usart_rx_sta & 0x3FFF;
            g_usart_rx_buf[rx_len] = '\0';

            if (cmd_parse(g_usart_rx_buf, rx_len, &cmd))
            {
                int start = 0, end = MOTOR_COUNT;
                if (cmd.motor_id > 0) {
                    start = (int)s_motor_map[cmd.motor_id];
                    end = start + 1;
                    LOGW("Cmd received for motor %d, stopping...", cmd.motor_id);
                } else {
                    LOGW("Cmd received, stopping all motors...");
                }

                for (int i = start; i < end; i++) {
                    motor_set_speed((motor_id_t)i, 0);
                    speed_filtered[i] = 0.0f;
                }

                apply_params(pid, &filter_alpha, &cmd, tgt_speed_mm_s);

                for (int i = start; i < end; i++) {
                    pid_reset(&pid[i]);
                    motor_reset_encoder((motor_id_t)i);
                }
                delay_ms(3000);

                for (int i = start; i < end; i++) {
                    last_enc[i] = motor_get_encoder((motor_id_t)i);
                }
                last_tick = HAL_GetTick();
                LOGW("Motors restarted.");
            }
            else
            {
                LOGW("Invalid cmd: %s", g_usart_rx_buf);
            }

            g_usart_rx_sta = 0;
        }

        if (HAL_GetTick() - last_tick >= PID_PERIOD_MS)
        {
            last_tick += PID_PERIOD_MS;

            //便利所有电机
            for (int i = 0; i < MOTOR_COUNT; i++) {
                float setpoint = tgt_speed_mm_s[i];

                //获取增量编码器值--编码器的累计值
                cur_enc = motor_get_encoder((motor_id_t)i);

                //计算速度脉冲
                speed_pulse = (float)(cur_enc - last_enc[i]);

                //更新上次电机编码器值
                last_enc[i] = cur_enc;

                //计算目标速度
                speed_raw_mm_s = pulses_per_period_to_speed(speed_pulse);

                //低通滤波目标速度
                // speed_filtered[i] = filter_alpha * speed_raw_mm_s
                //                   + (1.0f - filter_alpha) * speed_filtered[i];

                //计算pid输出
                pid_out = pid_calculate(&pid[i], setpoint, speed_raw_mm_s,
                                        PID_PERIOD_S);
                pwm_value = (int16_t)pid_out;
                
                //设置电机pwm值
                //if(i != 1) pwm_value = 0;
                motor_set_speed((motor_id_t)i, pwm_value);
                if (i == 1)
                LOGW("motor[%d] kp=%.1f,setpoint=%.1f, cur_speed=%.1f, pwm_value=%.0f,speed_pulse=%.1f",
                     i, pid[i].kp,setpoint, speed_raw_mm_s, pwm_value, speed_pulse);
                /* pwm_percent = (float)pwm_value / (float)MOTOR_SPEED_MAX * 100.0f; */
               /*  LOGI("[%d] raw=%.1f flt=%.1f err=%.1f | P=%.1f I=%.1f D=%.1f FF=%.1f | out=%.1f pwm=%.1f%%",
                     i,
                     speed_raw_mm_s,
                     speed_filtered[i],
                     pid[i].error,
                     pid[i].p_out,
                     pid[i].i_out,
                     pid[i].d_out,
                     pid[i].ff_out,
                     pid[i].output,
                     pwm_percent);
            } */
                }

            LED0_TOGGLE();
        }
    }
}