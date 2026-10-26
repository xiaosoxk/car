/**
 * attitude.c — 二维卡尔曼姿态解算: 状态 [θ(°), 陀螺零偏(°/s)]
 * 推导见手册 2.3 节: 预测=陀螺积分, 观测=加速度计 atan2 角, K 按方差比分配
 *   一维: K = P⁻/(P⁻+R) (最小方差), 二维推广见五公式逐项展开
 */
#include "attitude.h"
#include "balance_config.h"
#include <math.h>

static float s_theta = 0.0f, s_bias = 0.0f;
static float s_P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
static float s_gyro_raw = 0.0f;

void Att_Update(float gyro_dps, float ax_g, float az_g)
{
    s_gyro_raw = gyro_dps;
    float z = atan2f(-ax_g, az_g) * RAD2DEG;   /* 观测角: 前倾为正(°) */

    /* ---- 预测(公式①②): θ⁻=θ+(g-b)dt; P⁻=FPFᵀ+Q ---- */
    s_theta += CTRL_DT * (gyro_dps - s_bias);
    float p11 = s_P[0][0] + CTRL_DT * (CTRL_DT * s_P[1][1]
              - s_P[0][1] - s_P[1][0]) + KF_Q_ANGLE;
    float p12 = s_P[0][1] - CTRL_DT * s_P[1][1];
    float p21 = s_P[1][0] - CTRL_DT * s_P[1][1];
    float p22 = s_P[1][1] + KF_Q_BIAS;

    /* ---- 更新(公式③④⑤) ---- */
    float S  = p11 + KF_R_MEAS;                /* 新息方差 */
    float K1 = p11 / S, K2 = p21 / S;          /* 增益 */
    float y  = z - s_theta;                    /* 新息 */
    s_theta += K1 * y;
    s_bias  += K2 * y;                         /* 零偏在线自学习 */

    s_P[0][0] = p11 - K1 * p11;  s_P[0][1] = p12 - K1 * p12;
    s_P[1][0] = p21 - K2 * p11;  s_P[1][1] = p22 - K2 * p12;
}

float Att_GetTheta(void)    { return s_theta; }
float Att_GetThetaDot(void) { return s_gyro_raw; }
float Att_GetBias(void)     { return s_bias; }
