/**
 * controller.h — 控制器接口
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H

void  Controller_ResetIVel(void);
float Controller_Update(float theta_deg, float thdot_rps,
                        float s, float v, float v_ref);
/* 返回双轮合计力矩指令, N·m */

#endif /* CONTROLLER_H */
