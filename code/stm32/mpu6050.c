/**
 * mpu6050.c — MPU6050 驱动: 初始化 / 突发读取 / 零偏标定 / 总线恢复
 * I2C1: PB8=SCL, PB9=SDA, 400kHz, 上拉 4.7kΩ(上升时间计算见指南 3.2.3)
 * 符号约定: x 轴指向车头, z 轴向上, 前倾为正 —— 换安装方向需改 atan2 符号
 */
#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "balance_config.h"

extern I2C_HandleTypeDef hi2c1;

static float s_gyro_bias = 0.0f;     /* 上电标定的陀螺零偏(°/s) */

/* I2C 死锁恢复: 9 个 SCL 脉冲释放从机 SDA, 再重新初始化外设
 * 症状: 运行几分钟 MPU 数据断流 —— 主机读途中复位, 从机停在应答态 */
static void i2c_bus_recover(void)
{
    GPIO_InitTypeDef io = {0};
    io.Pin   = GPIO_PIN_8;
    io.Mode  = GPIO_MODE_OUTPUT_OD;
    io.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &io);
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 500; d++) { }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        for (volatile int d = 0; d < 500; d++) { }
    }
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_Init(&hi2c1);
}

static HAL_StatusTypeDef mpu_write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, 1, &val, 1, 4);
}

uint8_t MPU_Init(void)
{
    uint8_t id = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, MPU_WHO_AM_I, 1, &id, 1, 10) != HAL_OK
        || id != 0x68) {
        i2c_bus_recover();
        return 0;                    /* 自检失败: 上层停机 */
    }
    mpu_write(MPU_PWR_MGMT1, 0x01);  /* 唤醒, 时钟=PLL(陀螺), 比内部RC稳 */
    HAL_Delay(10);
    mpu_write(MPU_DLPF_CFG, 0x03);   /* 内部低通42Hz: 压机械振动, 滞后可控 */
    mpu_write(MPU_GYRO_CFG, 0x18);   /* ±2000dps -> 16.4 LSB/(°/s) */
    mpu_write(MPU_ACCEL_CFG, 0x00);  /* ±2g -> 16384 LSB/g */

    /* 零偏标定: 上电必须静止, 400 样本均值; 运动中标定=污染零偏 */
    uint8_t b[14];
    float sum = 0.0f;
    for (int i = 0; i < 400; i++) {
        if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, MPU_DATA, 1, b, 14, 4) != HAL_OK)
            return 0;
        int16_t gy = (int16_t)((b[10] << 8) | b[11]);
        sum += (float)gy / GYRO_LSB;
        HAL_Delay(2);
    }
    s_gyro_bias = sum / 400.0f;
    return 1;
}

/* 突发读 14 字节: 一次事务保证加速度/陀螺属于同一采样时刻(防撕裂) */
uint8_t MPU_Read(MpuFrame *f)
{
    uint8_t b[14];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, MPU_DATA, 1, b, 14, 4) != HAL_OK)
        return 0;                    /* 调用方立即断力矩 */
    f->ax_g     = (float)(int16_t)((b[0] << 8) | b[1]) / ACC_LSB;
    f->az_g     = (float)(int16_t)((b[4] << 8) | b[5]) / ACC_LSB;
    f->gyro_dps = (float)(int16_t)((b[10] << 8) | b[11]) / GYRO_LSB - s_gyro_bias;
    return 1;
}

float MPU_GetBias(void) { return s_gyro_bias; }
