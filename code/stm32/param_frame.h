/**
 * param_frame.h — 上位机参数帧接收接口
 */
#ifndef PARAM_FRAME_H
#define PARAM_FRAME_H

#include <stdint.h>

void Param_Poll(float *v_ref);   /* 主循环调用; 解析 TYPE=0x02 帧(float v_ref) */

#endif /* PARAM_FRAME_H */
