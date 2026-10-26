/**
 * encoder.c — TIM3/TIM4 编码器模式(4倍频) -> 车速/位移
 * 量化分析见手册 4.5.3: 5ms 内 1 计数 = 0.024m/s, 必须低通
 */
#include "stm32f4xx_hal.h"
#include "encoder.h"
#include "balance_config.h"

extern TIM_HandleTypeDef htim3, htim4;

static float v_lp = 0.0f, s_pos = 0.0f;

void Encoder_Start(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

float Encoder_GetSpeed(void)          /* m/s, 左右轮平均, 已低通 */
{
    /* 强转 int16_t: 16位计数器回绕自动成有符号差值 */
    int32_t dn = (int16_t)__HAL_TIM_GET_COUNTER(&htim3)
               + (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    float v = 2.0f * 3.14159f * WHEEL_R * (float)dn / (ENC_PPR * 0.005f);
    v_lp  += VEL_LP_A * (v - v_lp);   /* 一阶低通: 压量化噪声 */
    s_pos += v_lp * CTRL_DT;
    return v_lp;
}

float Encoder_GetPos(void)  { return s_pos; }
void  Encoder_ResetPos(void){ s_pos = 0.0f; }
