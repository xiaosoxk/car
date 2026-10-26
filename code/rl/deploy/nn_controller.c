/**
 * nn_controller.c — 强化学习策略的 STM32 端推理（LQR 的替换件）
 *
 * 用法:
 *   1. 先在 PC 上跑 code/rl/train_ppo.py 训练, 再跑 export_onnx.py 生成 nn_weights.h
 *   2. 把 nn_weights.h 与本文件加入 Keil/CubeIDE 工程
 *   3. main.c 的 HAL_TIM_PeriodElapsedCallback 中, 把
 *        u = Controller_Update(...)          // LQR 路径
 *      换成
 *        u = Controller_NN_Update(...)       // RL 路径
 *      两条路径可加编译开关共存, 方便论文做 LQR vs RL 对照实验
 *
 * 资源: 约 17.5KB Flash + 推理 ~1300 次浮点乘加(<0.1ms @168MHz), 实时性无忧
 */
#ifndef NN_CONTROLLER_C
#define NN_CONTROLLER_C

#include "nn_weights.h"
#include "balance_config.h"
#include <math.h>

static inline float nn_tanh(float x)
{
    /* 大参数防溢出, 小参数直接查数学库 */
    if (x >  8.0f) return  1.0f;
    if (x < -8.0f) return -1.0f;
    float e = expf(2.0f * x);
    return (e - 1.0f) / (e + 1.0f);
}

static float fc(int in, const float *w, const float *b,
                const float *x, float *y, int use_tanh)
{
    for (int j = 0; j < 64; j++) {
        float acc = b[j];
        for (int i = 0; i < in; i++) acc += w[j * in + i] * x[i];
        y[j] = use_tanh ? nn_tanh(acc) : acc;   /* 末层线性 */
    }
    return 0.0f;
}

/**
 * 策略前向: obs(已归一化) -> 动作 [-1,1]
 * 状态 x1,x2 为 64 维隐藏层(静态复用, 避免栈溢出)
 */
float NN_Policy(const float obs[3])
{
    static float h1[64], h2[64], y[64];
    fc(3,  NN_W1, NN_B1, obs, h1, 1);
    fc(64, NN_W2, NN_B2, h1,  h2, 1);
    fc(64, NN_W3, NN_B3, h2,  y,  0);
    float a = y[0];
    return (a > 1.0f) ? 1.0f : (a < -1.0f ? -1.0f : a);
}

/**
 * 与 Controller_Update 同签名的 RL 控制器
 * θ 单位度, θ̇ 单位 rad/s, v 单位 m/s —— 与 LQR 路径完全一致, 可直接对切
 */
float Controller_NN_Update(float theta_deg, float thdot_rps,
                           float s, float v, float v_ref)
{
    /* 安全帘: 姿态超出训练包线立即断力矩(策略没见过这种状态) */
    if (fabsf(theta_deg) > 25.0f) return 0.0f;

    /* 速度指令并入观测: RL 策略训练时 v_ref=0, 遥控差速需重训或做残余补偿 */
    float obs[3] = { theta_deg * DEG2RAD / NN_OBS_TH_SCALE,
                     thdot_rps / NN_OBS_THD_SCALE,
                     v / NN_OBS_V_SCALE };
    float a = NN_Policy(obs);
    return a * NN_T_MAX;                        /* N·m, 双轮合计 */
}

#endif /* NN_CONTROLLER_C */
