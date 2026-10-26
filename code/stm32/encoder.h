/**
 * encoder.h — 编码器测速接口
 */
#ifndef ENCODER_H
#define ENCODER_H

void  Encoder_Start(void);
float Encoder_GetSpeed(void);   /* m/s, 左右轮平均, 已一阶低通 */
float Encoder_GetPos(void);     /* 位移 m */
void  Encoder_ResetPos(void);

#endif /* ENCODER_H */
