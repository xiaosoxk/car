/**
 * uart_frame.h — 遥测/参数帧协议接口
 * 帧结构: AA 55 | TYPE(1) | LEN(1) | SEQ(1) | PAYLOAD | CRC16(低字节在前)
 */
#ifndef UART_FRAME_H
#define UART_FRAME_H

#include <stdint.h>

#define FRAME_SYNC0   0xAAU
#define FRAME_SYNC1   0x55U

uint16_t CRC16_Modbus(const uint8_t *p, uint32_t n);
void Telem_Send(float th, float thd, float v, float u, float bias);

#endif /* UART_FRAME_H */
