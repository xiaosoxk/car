/**
 * motor.h — 电机驱动接口
 */
#ifndef MOTOR_H
#define MOTOR_H

void Motor_Start(void);
void Motor_SetTorque(float u);   /* u: 双轮合计力矩 N·m, 内部含归一化+死区补偿 */

#endif /* MOTOR_H */
