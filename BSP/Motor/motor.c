/**
 ****************************************************************************************************
 * @file        motor.c
 * @brief       ???????????(PWM + ???? + ??????)
 *
 * ???????:
 *   LF: PWM PA8   TIM1_CH1    IN1 PC0  IN2 PC1  ENC PA5  PA1   TIM2
 *   RF: PWM PE5   TIM9_CH1    IN1 PC2  IN2 PC3  ENC PA6  PA7   TIM3
 *   LR: PWM PF6   TIM10_CH1   IN1 PC4  IN2 PC5  ENC PD12 PD13  TIM4
 *   RR: PWM PF8   TIM13_CH1   IN1 PD2  IN2 PG11 ENC PC6  PC7   TIM8
 *
 * PWM???: 20kHz
 * ??????: -1000 ~ +1000 (??=???, ??=???)
 * ????????: TI12 (4???)
 ****************************************************************************************************
 */

#include "motor.h"

#define PWM_FREQ_HZ             20000

#define PWM_ARR_APB2            8399
#define PWM_ARR_APB1            4199
#define FILTER_USER             0

/* ============== LF ??????? ============== */
#define LF_PWM_PORT             GPIOA       /* PA8  */
#define LF_PWM_PIN              GPIO_PIN_8  /* PA8  */
#define LF_PWM_AF               GPIO_AF1_TIM1

#define LF_IN1_PORT             GPIOC       /* PC0  */
#define LF_IN1_PIN              GPIO_PIN_0  /* PC0  */
#define LF_IN2_PORT             GPIOC       /* PC1  */
#define LF_IN2_PIN              GPIO_PIN_1  /* PC1  */

#define LF_ENC_PORT1            GPIOA       /* PA5  */
#define LF_ENC_PIN1             GPIO_PIN_5  /* PA5  */
#define LF_ENC_PORT2            GPIOA       /* PA1  */
#define LF_ENC_PIN2             GPIO_PIN_1  /* PA1  */
#define LF_ENC_AF               GPIO_AF1_TIM2

/* ============== RF ??????? ============== */
#define RF_PWM_PORT             GPIOE       /* PE5  */
#define RF_PWM_PIN              GPIO_PIN_5  /* PE5  */
#define RF_PWM_AF               GPIO_AF3_TIM9

#define RF_IN1_PORT             GPIOC       /* PC2  */
#define RF_IN1_PIN              GPIO_PIN_2  /* PC2  */
#define RF_IN2_PORT             GPIOC       /* PC3  */
#define RF_IN2_PIN              GPIO_PIN_3  /* PC3  */

#define RF_ENC_PORT1            GPIOA       /* PA6  */
#define RF_ENC_PIN1             GPIO_PIN_6  /* PA6  */
#define RF_ENC_PORT2            GPIOA       /* PA7  */
#define RF_ENC_PIN2             GPIO_PIN_7  /* PA7  */
#define RF_ENC_AF               GPIO_AF2_TIM3

/* ============== LR ??????? ============== */
#define LR_PWM_PORT             GPIOF       /* PF6  */
#define LR_PWM_PIN              GPIO_PIN_6  /* PF6  */
#define LR_PWM_AF               GPIO_AF3_TIM10

#define LR_IN1_PORT             GPIOC       /* PC4  */
#define LR_IN1_PIN              GPIO_PIN_4  /* PC4  */
#define LR_IN2_PORT             GPIOC       /* PC5  */
#define LR_IN2_PIN              GPIO_PIN_5  /* PC5  */

#define LR_ENC_PORT1            GPIOD       /* PD12 */
#define LR_ENC_PIN1             GPIO_PIN_12 /* PD12 */
#define LR_ENC_PORT2            GPIOD       /* PD13 */
#define LR_ENC_PIN2             GPIO_PIN_13 /* PD13 */
#define LR_ENC_AF               GPIO_AF2_TIM4

/* ============== RR ??????? ============== */
#define RR_PWM_PORT             GPIOF       /* PF8  */
#define RR_PWM_PIN              GPIO_PIN_8  /* PF8  */
#define RR_PWM_AF               GPIO_AF9_TIM13

#define RR_IN1_PORT             GPIOD       /* PD2  */
#define RR_IN1_PIN              GPIO_PIN_2  /* PD2  */
#define RR_IN2_PORT             GPIOG       /* PG11 */
#define RR_IN2_PIN              GPIO_PIN_11 /* PG11 */

#define RR_ENC_PORT1            GPIOC       /* PC6  */
#define RR_ENC_PIN1             GPIO_PIN_6  /* PC6  */
#define RR_ENC_PORT2            GPIOC       /* PC7  */
#define RR_ENC_PIN2             GPIO_PIN_7  /* PC7  */
#define RR_ENC_AF               GPIO_AF3_TIM8

/* ============== ???????? ============== */
static TIM_HandleTypeDef htim1;
static TIM_HandleTypeDef htim9;
static TIM_HandleTypeDef htim10;
static TIM_HandleTypeDef htim13;

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4;
static TIM_HandleTypeDef htim8;

/* ============== ???????????(?????) ============== */
static int32_t  s_enc_accum[MOTOR_COUNT] = {0};
static uint32_t s_enc_last[MOTOR_COUNT] = {0};

static void pwm_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = LF_PWM_PIN;
    gpio.Alternate = LF_PWM_AF;
    HAL_GPIO_Init(LF_PWM_PORT, &gpio);

    gpio.Pin = RF_PWM_PIN;
    gpio.Alternate = RF_PWM_AF;
    HAL_GPIO_Init(RF_PWM_PORT, &gpio);

    gpio.Pin = LR_PWM_PIN;
    gpio.Alternate = LR_PWM_AF;
    HAL_GPIO_Init(LR_PWM_PORT, &gpio);

    gpio.Pin = RR_PWM_PIN;
    gpio.Alternate = RR_PWM_AF;
    HAL_GPIO_Init(RR_PWM_PORT, &gpio);
}

static void pwm_timer_init(void)
{
    TIM_OC_InitTypeDef oc = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM9_CLK_ENABLE();
    __HAL_RCC_TIM10_CLK_ENABLE();
    __HAL_RCC_TIM13_CLK_ENABLE();

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PWM_ARR_APB2;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    htim9.Instance = TIM9;
    htim9.Init.Prescaler = 0;
    htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim9.Init.Period = PWM_ARR_APB2;
    htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim9);
    HAL_TIM_PWM_ConfigChannel(&htim9, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);

    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 0;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = PWM_ARR_APB2;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim10);
    HAL_TIM_PWM_ConfigChannel(&htim10, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    htim13.Instance = TIM13;
    htim13.Init.Prescaler = 0;
    htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim13.Init.Period = PWM_ARR_APB1;
    htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim13);
    HAL_TIM_PWM_ConfigChannel(&htim13, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
}

static void dir_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = LF_IN1_PIN;
    HAL_GPIO_Init(LF_IN1_PORT, &gpio);
    gpio.Pin = LF_IN2_PIN;
    HAL_GPIO_Init(LF_IN2_PORT, &gpio);

    gpio.Pin = RF_IN1_PIN;
    HAL_GPIO_Init(RF_IN1_PORT, &gpio);
    gpio.Pin = RF_IN2_PIN;
    HAL_GPIO_Init(RF_IN2_PORT, &gpio);

    gpio.Pin = LR_IN1_PIN;
    HAL_GPIO_Init(LR_IN1_PORT, &gpio);
    gpio.Pin = LR_IN2_PIN;
    HAL_GPIO_Init(LR_IN2_PORT, &gpio);

    gpio.Pin = RR_IN1_PIN;
    HAL_GPIO_Init(RR_IN1_PORT, &gpio);
    gpio.Pin = RR_IN2_PIN;
    HAL_GPIO_Init(RR_IN2_PORT, &gpio);

    HAL_GPIO_WritePin(LF_IN1_PORT, LF_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LF_IN2_PORT, LF_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RF_IN1_PORT, RF_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RF_IN2_PORT, RF_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LR_IN1_PORT, LR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LR_IN2_PORT, LR_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RR_IN1_PORT, RR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RR_IN2_PORT, RR_IN2_PIN, GPIO_PIN_RESET);
}

static void enc_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = LF_ENC_PIN1;
    gpio.Alternate = LF_ENC_AF;
    HAL_GPIO_Init(LF_ENC_PORT1, &gpio);
    gpio.Pin = LF_ENC_PIN2;
    HAL_GPIO_Init(LF_ENC_PORT2, &gpio);

    gpio.Pin = RF_ENC_PIN1;
    gpio.Alternate = RF_ENC_AF;
    HAL_GPIO_Init(RF_ENC_PORT1, &gpio);
    gpio.Pin = RF_ENC_PIN2;
    HAL_GPIO_Init(RF_ENC_PORT2, &gpio);

    gpio.Pin = LR_ENC_PIN1;
    gpio.Alternate = LR_ENC_AF;
    HAL_GPIO_Init(LR_ENC_PORT1, &gpio);
    gpio.Pin = LR_ENC_PIN2;
    HAL_GPIO_Init(LR_ENC_PORT2, &gpio);

    gpio.Pin = RR_ENC_PIN1;
    gpio.Alternate = RR_ENC_AF;
    HAL_GPIO_Init(RR_ENC_PORT1, &gpio);
    gpio.Pin = RR_ENC_PIN2;
    HAL_GPIO_Init(RR_ENC_PORT2, &gpio);
}

static void enc_timer_init(void)
{
    TIM_Encoder_InitTypeDef enc1 = {0};
    TIM_Encoder_InitTypeDef enc2 = {0};
    TIM_Encoder_InitTypeDef enc3 = {0};
    TIM_Encoder_InitTypeDef enc4 = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    enc1.EncoderMode = TIM_ENCODERMODE_TI12;
    enc1.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc1.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc1.IC1Prescaler = TIM_ICPSC_DIV1;
    enc1.IC1Filter = FILTER_USER;
    enc1.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc1.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc1.IC2Prescaler = TIM_ICPSC_DIV1;
    enc1.IC2Filter = FILTER_USER;

    enc2.EncoderMode = TIM_ENCODERMODE_TI12;
    enc2.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc2.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc2.IC1Prescaler = TIM_ICPSC_DIV1;
    enc2.IC1Filter = FILTER_USER;
    enc2.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc2.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc2.IC2Prescaler = TIM_ICPSC_DIV1;
    enc2.IC2Filter = FILTER_USER;

    enc3.EncoderMode = TIM_ENCODERMODE_TI12;
    enc3.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc3.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc3.IC1Prescaler = TIM_ICPSC_DIV1;
    enc3.IC1Filter = FILTER_USER;
    enc3.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc3.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc3.IC2Prescaler = TIM_ICPSC_DIV1;
    enc3.IC2Filter = FILTER_USER;

    enc4.EncoderMode = TIM_ENCODERMODE_TI12;
    enc4.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc4.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc4.IC1Prescaler = TIM_ICPSC_DIV1;
    enc4.IC1Filter = FILTER_USER;
    enc4.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc4.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc4.IC2Prescaler = TIM_ICPSC_DIV1;
    enc4.IC2Filter = FILTER_USER;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Encoder_Init(&htim2, &enc1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Encoder_Init(&htim3, &enc2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFF;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Encoder_Init(&htim4, &enc3);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 0xFFFF;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Encoder_Init(&htim8, &enc4);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

void motor_init(void)
{
    pwm_gpio_init();
    pwm_timer_init();
    dir_gpio_init();
    enc_gpio_init();
    enc_timer_init();

    s_enc_last[MOTOR_LF] = __HAL_TIM_GET_COUNTER(&htim2);
    s_enc_last[MOTOR_RF] = __HAL_TIM_GET_COUNTER(&htim3);
    s_enc_last[MOTOR_LR] = __HAL_TIM_GET_COUNTER(&htim4);
    s_enc_last[MOTOR_RR] = __HAL_TIM_GET_COUNTER(&htim8);
}

void motor_set_speed(motor_id_t id, int16_t speed)
{
    uint32_t pulse = 0;

    switch (id)
    {
    case MOTOR_LF:
        if(speed > 0){
            HAL_GPIO_WritePin(LF_IN1_PORT, LF_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LF_IN2_PORT, LF_IN2_PIN, GPIO_PIN_RESET);
        }else if(speed < 0){
            HAL_GPIO_WritePin(LF_IN1_PORT, LF_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LF_IN2_PORT, LF_IN2_PIN, GPIO_PIN_SET);
            speed = -speed;
        }else{
            HAL_GPIO_WritePin(LF_IN1_PORT, LF_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LF_IN2_PORT, LF_IN2_PIN, GPIO_PIN_RESET);
        }
        pulse = (uint32_t)speed * PWM_ARR_APB2 / MOTOR_SPEED_MAX;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
        break;
    case MOTOR_RF:
        if(speed > 0){
            HAL_GPIO_WritePin(RF_IN1_PORT, RF_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RF_IN2_PORT, RF_IN2_PIN, GPIO_PIN_SET);
        }else if(speed < 0){
            HAL_GPIO_WritePin(RF_IN1_PORT, RF_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(RF_IN2_PORT, RF_IN2_PIN, GPIO_PIN_RESET);
            speed = -speed;
        }else{
            HAL_GPIO_WritePin(RF_IN1_PORT, RF_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RF_IN2_PORT, RF_IN2_PIN, GPIO_PIN_RESET);
        }
        pulse = (uint32_t)speed * PWM_ARR_APB2 / MOTOR_SPEED_MAX;
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pulse);
        break;
    case MOTOR_LR:
        if(speed > 0){
            HAL_GPIO_WritePin(LR_IN1_PORT, LR_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LR_IN2_PORT, LR_IN2_PIN, GPIO_PIN_RESET);
        }else if(speed < 0){
            HAL_GPIO_WritePin(LR_IN1_PORT, LR_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LR_IN2_PORT, LR_IN2_PIN, GPIO_PIN_SET);
            speed = -speed;
        }else{
            HAL_GPIO_WritePin(LR_IN1_PORT, LR_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LR_IN2_PORT, LR_IN2_PIN, GPIO_PIN_RESET);
        }
        pulse = (uint32_t)speed * PWM_ARR_APB2 / MOTOR_SPEED_MAX;
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pulse);
        break;
    case MOTOR_RR:
        if(speed > 0){
            HAL_GPIO_WritePin(RR_IN1_PORT, RR_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RR_IN2_PORT, RR_IN2_PIN, GPIO_PIN_SET);
        }else if(speed < 0){
            HAL_GPIO_WritePin(RR_IN1_PORT, RR_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(RR_IN2_PORT, RR_IN2_PIN, GPIO_PIN_RESET);
            speed = -speed;
        }else{
            HAL_GPIO_WritePin(RR_IN1_PORT, RR_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RR_IN2_PORT, RR_IN2_PIN, GPIO_PIN_RESET);
        }
        pulse = (uint32_t)speed * PWM_ARR_APB1 / MOTOR_SPEED_MAX;
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, pulse);
        break;
    default: break;
    }
}

void motor_encoder_update(void)
{
    uint32_t cur[MOTOR_COUNT];

    cur[MOTOR_LF] = __HAL_TIM_GET_COUNTER(&htim2);
    cur[MOTOR_RF] = __HAL_TIM_GET_COUNTER(&htim3);
    cur[MOTOR_LR] = __HAL_TIM_GET_COUNTER(&htim4);
    cur[MOTOR_RR] = __HAL_TIM_GET_COUNTER(&htim8);
    LOGI("cur[MOTOR_LF]=%d, cur[MOTOR_RF]=%d, cur[MOTOR_LR]=%d, cur[MOTOR_RR]=%d", cur[MOTOR_LF], cur[MOTOR_RF], cur[MOTOR_LR], cur[MOTOR_RR]);


    s_enc_accum[MOTOR_LF] += (int32_t)(cur[MOTOR_LF] - s_enc_last[MOTOR_LF]);
    

    s_enc_accum[MOTOR_RF] += (int16_t)(uint16_t)(cur[MOTOR_RF] - s_enc_last[MOTOR_RF]);
    static int count = 0;
    if(count++ % 20 == 0)
    LOGI("cur[MOTOR_RF]=%d, s_enc_last[MOTOR_RF]=%d", cur[MOTOR_RF], s_enc_last[MOTOR_RF]);
    s_enc_accum[MOTOR_LR] += (int16_t)(uint16_t)(cur[MOTOR_LR] - s_enc_last[MOTOR_LR]);
    s_enc_accum[MOTOR_RR] += (int16_t)(uint16_t)(cur[MOTOR_RR] - s_enc_last[MOTOR_RR]);

    s_enc_last[MOTOR_LF] = cur[MOTOR_LF];
    s_enc_last[MOTOR_RF] = cur[MOTOR_RF];
    s_enc_last[MOTOR_LR] = cur[MOTOR_LR];
    s_enc_last[MOTOR_RR] = cur[MOTOR_RR];
}

int32_t motor_get_encoder(motor_id_t id)
{
    if (id >= MOTOR_COUNT) return 0;
    return s_enc_accum[id];
}

void motor_reset_encoder(motor_id_t id)
{
    if (id >= MOTOR_COUNT) return;
    s_enc_accum[id] = 0;
    switch (id)
    {
    case MOTOR_LF: __HAL_TIM_SET_COUNTER(&htim2, 0); s_enc_last[MOTOR_LF] = 0; break;
    case MOTOR_RF: __HAL_TIM_SET_COUNTER(&htim3, 0); s_enc_last[MOTOR_RF] = 0; break;
    case MOTOR_LR: __HAL_TIM_SET_COUNTER(&htim4, 0); s_enc_last[MOTOR_LR] = 0; break;
    case MOTOR_RR: __HAL_TIM_SET_COUNTER(&htim8, 0); s_enc_last[MOTOR_RR] = 0; break;
    default: break;
    }
}

int32_t motor_get_output_rev(motor_id_t id)
{
    return motor_get_encoder(id) / ENCODER_PULSES_PER_REV;
}

int32_t motor_get_output_angle(motor_id_t id)
{
    return (int32_t)((int64_t)motor_get_encoder(id) * 360 / ENCODER_PULSES_PER_REV);
}