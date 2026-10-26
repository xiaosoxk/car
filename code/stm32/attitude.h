/**
 * attitude.h — 二维卡尔曼姿态解算接口
 */
#ifndef ATTITUDE_H
#define ATTITUDE_H

void  Att_Update(float gyro_dps, float ax_g, float az_g);
float Att_GetTheta(void);     /* 倾角, 度, 前倾为正 */
float Att_GetThetaDot(void);  /* 角速度, °/s, 直接用去零偏陀螺(免微分噪声) */
float Att_GetBias(void);      /* 卡尔曼在线学出的陀螺零偏 */

#endif /* ATTITUDE_H */
