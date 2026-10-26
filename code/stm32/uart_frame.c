/**
 * uart_frame.c — 遥测帧: AA 55 | TYPE | LEN | SEQ | PAYLOAD | CRC16(低前)
 * USART1 921600 8N1, DMA 发送; CRC 对 TYPE..PAYLOAD 全部字节
 * 带宽: 27字节×100Hz ≈ 27kbps, 仅占 921600bps 的 3%
 */
#include "stm32f4xx_hal.h"
#include "uart_frame.h"
#include <string.h>

extern UART_HandleTypeDef huart1;
static uint8_t s_seq = 0, s_div = 0;

uint16_t CRC16_Modbus(const uint8_t *p, uint32_t n)
{
    uint16_t c = 0xFFFF;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++)
            c = (c & 1U) ? (uint16_t)((c >> 1) ^ 0xA001U) : (uint16_t)(c >> 1);
    }
    return c;
}

/* 载荷: θ(°) θ̇(rad/s) v(m/s) u(N·m) KF零偏(°/s) — 5×float = 20 字节 */
void Telem_Send(float th, float thd, float v, float u, float bias)
{
    if (++s_div < 2) return;              /* 200Hz -> 100Hz */
    s_div = 0;
    static uint8_t fr[27];
    fr[0] = FRAME_SYNC0; fr[1] = FRAME_SYNC1;
    fr[2] = 0x01;                         /* TYPE=遥测 */
    fr[3] = 20;                           /* LEN */
    fr[4] = s_seq++;
    memcpy(&fr[5],  &th,   4);  memcpy(&fr[9],  &thd,  4);
    memcpy(&fr[13], &v,    4);  memcpy(&fr[17], &u,    4);
    memcpy(&fr[21], &bias, 4);
    uint16_t crc = CRC16_Modbus(&fr[2], 23);
    fr[25] = (uint8_t)(crc & 0xFF);
    fr[26] = (uint8_t)(crc >> 8);
    HAL_UART_Transmit_DMA(&huart1, fr, sizeof fr);
}
