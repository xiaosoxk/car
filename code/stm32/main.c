/**
 * main.c — 调度核心: TIM6 5ms 控制中断(硬实时) + 主循环(gating/看门狗/在线参数)
 * 外设: I2C1(MPU) TIM1(20kHz PWM) TIM3/4(编码器) TIM6(5ms) USART1(遥测) ADC1(VBAT)
 * CubeMX 配置逐项步骤见 docs/制作指南.md 第3章
 */
#include "stm32f4xx_hal.h"
#include "balance_config.h"
#include "mpu6050.h"
#include "attitude.h"
#include "encoder.h"
#include "controller.h"
#include "motor.h"
#include "uart_frame.h"
#include "param_frame.h"
#include <math.h>

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1, htim3, htim4, htim6;
UART_HandleTypeDef huart1;
ADC_HandleTypeDef hadc1;
IWDG_HandleTypeDef hiwdg;

static volatile uint8_t g_run = 0;     /* gating 通过后才闭环 */
static float g_v_ref = 0.0f;           /* 目标速度, 上位机参数帧下发 */
static uint16_t g_idle = 0;

/* ---- 5ms 心跳: 控制环在中断内一次做完, 不允许回主循环再算 ----
 * 时序预算: I2C 0.40ms + 卡尔曼 0.02ms + 其余 <0.03ms, 裕量 91% */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *h)
{
    if (h->Instance != TIM6) return;
    MpuFrame f;
    if (!g_run || !MPU_Read(&f)) {     /* 未就绪 / I2C失败: 故障安全断力矩 */
        Motor_SetTorque(0.0f);
        return;
    }
    Att_Update(f.gyro_dps, f.ax_g, f.az_g);
    float v = Encoder_GetSpeed();
    float u = Controller_Update(Att_GetTheta(), Att_GetThetaDot() * DEG2RAD,
                                Encoder_GetPos(), v, g_v_ref);
    Motor_SetTorque(u);
    Telem_Send(Att_GetTheta(), Att_GetThetaDot() * DEG2RAD, v, u, Att_GetBias());
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();              /* CubeMX 生成: HSE 8MHz -> 168MHz */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_DMA_Init();

    if (!MPU_Init()) Error_Loop();     /* WHO_AM_I/零偏标定失败即停 */
    Encoder_Start();
    Motor_Start();

    hiwdg.Instance = IWDG;             /* 独立看门狗 ~500ms */
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload    = 625;
    HAL_IWDG_Init(&hiwdg);

    HAL_TIM_Base_Start_IT(&htim6);     /* 启动 200Hz 心跳 */
    while (1) {
        /* 启动 gating: 陀螺安静 + 车体近竖直, 持续 200ms 才放手 */
        if (!g_run && fabsf(Att_GetThetaDot()) < 5.0f
                   && fabsf(Att_GetTheta())  < 2.0f) {
            if (++g_idle > 40) {       /* 40×5ms = 200ms */
                g_run = 1;
                Controller_ResetIVel();
            }
        } else if (!g_run) {
            g_idle = 0;
        }
        Param_Poll(&g_v_ref);          /* 上位机参数帧解析(在线调参) */
        HAL_IWDG_Refresh(&hiwdg);
    }
}

void Error_Loop(void)                  /* 默认态=静止: 故障安全的底线 */
{
    Motor_SetTorque(0.0f);
    while (1) { }
}
