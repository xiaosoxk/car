/**
 * balance_config.h — 系统核心参数（全工程唯一参数源）
 * 平台: STM32F407VET6 + MPU6050 + TB6612FNG + GA25-370(1:30, AB相13线编码器)
 * 每个数值的推导/标定出处见注释；改参数只改这里。
 * 理论推导详见 docs/双轮自平衡机器人-毕业设计完整方案与答辩手册.docx
 */
#ifndef BALANCE_CONFIG_H
#define BALANCE_CONFIG_H

/* ---- 控制周期 ---- */
#define CTRL_DT        0.005f     /* 5ms = 200Hz, TIM6 更新中断; 不稳定极点15.7rad/s -> 时间尺度191ms */

/* ---- LQR 增益: code/design/design_lqr.m 离线解出, Q=diag(50,2,1,100) R=1, θ 单位 rad ---- */
#define K_THETA  (-2.798f)        /* 负号不是错误! θ>0前倾需u>0向前, 而公式为 u=-K·x */
#define K_THDOT  (-0.326f)        /* 单位: N·m / (rad/s) */
#define K_POS    (-0.094f)        /* 单位: N·m / m */
#define K_VEL    (-0.971f)        /* 单位: N·m / (m/s) */

/* ---- 卡尔曼滤波(单位: 度), R 由静止实测方差确定 ---- */
#define KF_Q_ANGLE  0.001f        /* 过程噪声: 陀螺积分失配方差 */
#define KF_Q_BIAS   0.003f        /* 零偏随机游走强度 */
#define KF_R_MEAS   0.03f         /* 观测噪声: acc角方差(静止5000点实测) */

/* ---- 速度环 PI(输出: 目标角偏置, 度) —— 先仰后动机理 ---- */
#define VEL_KP       0.9f
#define VEL_KI       0.35f
#define VEL_OUT_MAX  8.0f         /* 偏置限幅: 直立环能力的约1/3 */
#define VEL_LP_A     0.25f        /* 编码器速度一阶低通: 5ms内1计数=0.024m/s 量化噪声 */

/* ---- 执行器 ---- */
#define PWM_PERIOD   4199U        /* 84MHz/20kHz-1, TIM1 ARR */
#define U_MAX        0.95f        /* 归一化输出限幅 */
#define DEADZONE     0.06f        /* 实测死区占空比6%: 直立环关闭, 占空比从1%步进到轮动 */
#define TORQUE_FULL  0.55f        /* 归一化1.0对应的双轮合计力矩 N·m */
#define VBAT_NOM     7.4f         /* 电池标称电压, 输出归一化基准 */

/* ---- 传感 ---- */
#define WHEEL_R      0.03f        /* 轮半径 m */
#define ENC_PPR      1560.0f      /* 13线×4倍频×30减速比 */
#define GYRO_LSB     16.4f        /* ±2000dps: 16.4 LSB/(°/s) */
#define ACC_LSB      16384.0f     /* ±2g: 16384 LSB/g */
#define DEG2RAD      0.0174533f
#define RAD2DEG      57.29578f

/* ---- MPU6050 寄存器 ---- */
#define MPU_ADDR       (0x68U << 1)
#define MPU_WHO_AM_I   0x75U
#define MPU_PWR_MGMT1  0x6BU
#define MPU_DLPF_CFG   0x1AU
#define MPU_GYRO_CFG   0x1BU
#define MPU_ACCEL_CFG  0x1CU
#define MPU_DATA       0x3BU

#endif /* BALANCE_CONFIG_H */
