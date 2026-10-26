/**
 * param_frame.c — 上位机参数帧接收: 状态机 + CRC 校验
 * 协议与遥测帧一致(TYPE=0x02, 载荷 float v_ref), 任意切分下结果一致
 * USART1 RX 单字节中断进环形状态机, 主循环取走完整帧 —— 中断里不做任何解析
 */
#include "stm32f4xx_hal.h"
#include "param_frame.h"
#include "uart_frame.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

#define RX_BUF_LEN 32

static uint8_t s_rx_byte;
static uint8_t s_buf[RX_BUF_LEN];
static uint8_t s_idx = 0;
static volatile uint8_t s_frame_ready = 0;
static uint8_t s_frame[RX_BUF_LEN];

/* RX 单字节中断: 只搬运, 不解析 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance != USART1) return;
    if (s_idx < RX_BUF_LEN) s_buf[s_idx++] = s_rx_byte;
    /* 缓冲满/帧头不对时滑窗丢弃1字节, 状态机见主循环 */
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

static void state_machine(void)
{
    /* 滑窗找帧头 */
    while (s_idx >= 2 && (s_buf[0] != FRAME_SYNC0 || s_buf[1] != FRAME_SYNC1)) {
        memmove(s_buf, s_buf + 1, --s_idx);
    }
    if (s_idx < 4) return;                      /* 帧头+TYPE+LEN 未齐 */
    uint8_t need = 4 + s_buf[3] + 2;            /* 头4 + 载荷 + CRC */
    if (s_buf[3] > RX_BUF_LEN - 6) {            /* 非法长度: 丢1字节重扫 */
        memmove(s_buf, s_buf + 1, --s_idx);
        return;
    }
    if (s_idx < need) return;                   /* 半包: 等数据 */
    uint8_t len = s_buf[3];
    const uint8_t *p = &s_buf[2];
    uint16_t crc_recv = s_buf[4 + len] | (s_buf[5 + len] << 8);
    if (CRC16_Modbus(p, 2 + len) == crc_recv) {
        memcpy(s_frame, s_buf, need);
        s_frame_ready = 1;
    }
    memmove(s_buf, s_buf + need, RX_BUF_LEN - need);
    s_idx = (s_idx > need) ? (uint8_t)(s_idx - need) : 0;
}

void Param_Poll(float *v_ref)
{
    static uint8_t inited = 0;
    if (!inited) {
        inited = 1;
        HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
    }
    state_machine();
    if (s_frame_ready) {
        s_frame_ready = 0;
        if (s_frame[2] == 0x02 && s_frame[3] == 4) {   /* TYPE=0x02: v_ref */
            float val;
            memcpy(&val, &s_frame[4], 4);
            *v_ref = val;
        }
    }
}
