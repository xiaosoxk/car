/**
 * controller.c — LQR 状态反馈 + 速度 PI 串级(手册 4.3/4.5 节的实现)
 * 结构: bias = PI(v − v_ref);  u = −K·[θ+bias, θ̇, s, v]
 *       bias>0 等价于目标后仰 -> 减速; 反之加速。限幅 + 抗积分饱和。
 */
#include "controller.h"
#include "balance_config.h"

static float s_i_vel = 0.0f;

static float velocity_loop(float e)             /* e = v − v_ref, 超速为正 */
{
    s_i_vel += VEL_KI * e * CTRL_DT;            /* 积分: 吃坡度/死区静差 */
    if (s_i_vel >  VEL_OUT_MAX) s_i_vel =  VEL_OUT_MAX;   /* 抗饱和 */
    if (s_i_vel < -VEL_OUT_MAX) s_i_vel = -VEL_OUT_MAX;
    float bias = VEL_KP * e + s_i_vel;
    if (bias >  VEL_OUT_MAX) bias =  VEL_OUT_MAX;
    if (bias < -VEL_OUT_MAX) bias = -VEL_OUT_MAX;
    return bias;                                /* 单位: 度 */
}

void Controller_ResetIVel(void) { s_i_vel = 0.0f; }

float Controller_Update(float theta_deg, float thdot_rps,
                        float s, float v, float v_ref)
{
    float bias_deg = velocity_loop(v - v_ref);
    float th = (theta_deg + bias_deg) * DEG2RAD;   /* 度->rad, 不可省! */
    float u = -(K_THETA * th + K_THDOT * thdot_rps
              + K_POS * s + K_VEL * v);         /* N·m, 双轮合计 */
    return u;
}
