/**
 * motor.c — TB6612 驱动: 力矩指令 -> PWM
 * 死区补偿(手册7.4故障3) + 电池电压归一化(7.4故障8)
 * 接线: AIN1/PB12 AIN2/PB13 BIN1/PB14 BIN2/PB15
 *       PWMA=TIM1_CH1(PA8) PWMB=TIM1_CH2(PA9), STBY 接 3.3V
 */
#include "stm32f4xx_hal.h"
#include "motor.h"
#include "balance_config.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;

static float vbat_norm(void)              /* VBAT_NOM / VBAT, 分压 1/11 */
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    float vbat = (float)HAL_ADC_GetValue(&hadc1) * 3.3f / 4095.0f * 11.0f;
    return (vbat > 5.0f) ? VBAT_NOM / vbat : 1.0f;
}

static float deadband_comp(float d)       /* 小信号区抬到死区之上 */
{
    float mag = fabsf(d);
    float out = DEADZONE + (U_MAX - DEADZONE) * mag / U_MAX;
    return (d < 0.0f) ? -out : out;
}

static void ch_out(GPIO_TypeDef *p1, uint16_t g1,
                   GPIO_TypeDef *p2, uint16_t g2,
                   float d, uint32_t ch)
{
    uint32_t duty = (uint32_t)(fabsf(d) * PWM_PERIOD);
    GPIO_PinState s = (d >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(p1, g1, s);
    HAL_GPIO_WritePin(p2, g2, (GPIO_PinState)(s ^ GPIO_PIN_1));
    __HAL_TIM_SET_COMPARE(&htim1, ch, duty);
}

void Motor_SetTorque(float u)             /* u: 双轮合计力矩 N·m */
{
    float d = u / TORQUE_FULL;            /* 归一化 */
    d *= vbat_norm();                     /* 电压前馈: 出力不随电量缩水 */
    if (d >  U_MAX) d =  U_MAX;
    if (d < -U_MAX) d = -U_MAX;
    d = deadband_comp(d);
    ch_out(GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, d, TIM_CHANNEL_1);
    ch_out(GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, d, TIM_CHANNEL_2);
}

void Motor_Start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}
