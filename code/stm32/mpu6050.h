/**
 * mpu6050.h — MPU6050 驱动接口
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

typedef struct {
    float ax_g;        /* 加速度 x 轴, 单位 g */
    float az_g;        /* 加速度 z 轴, 单位 g */
    float gyro_dps;    /* 陀螺 y 轴角速度 °/s, 已减上电零偏 */
} MpuFrame;

uint8_t MPU_Init(void);              /* WHO_AM_I 自检 + DLPF/量程配置 + 静止零偏标定; 成功返回1 */
uint8_t MPU_Read(MpuFrame *f);       /* 14字节突发读; I2C失败返回0(调用方应断力矩) */
float   MPU_GetBias(void);           /* 当前零偏值(遥测用) */

#endif /* MPU6050_H */
