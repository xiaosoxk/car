# 强化学习扩展：从仿真训练到 STM32 部署

本目录是主项目的强化学习扩展：用 PPO 在与实车**同构**的仿真环境里训练平衡策略，导出后在 STM32F407 上做纯 float 前向推理，与 LQR 形成可对照的第二条控制路径。

## 0. 三条铁律（先读再跑）

1. **RL 不替代标定**。仿真环境和实车共用同一套机械参数（`env_balance.py` 顶部的常数 = `design_lqr.m` = `balance_config.h`），参数没实测就训 RL，等于在错的物理里学。
2. **先跑通 LQR 再碰 RL**。实车有 LQR 基线兜底，RL 才有对照、才有失败兜底（策略倒下就切回 LQR 重来）。
3. **未做域随机化的策略不准上实车**。第 4 节的 checklist 一项没勾就不许拆轮撑。

## 1. 目录

| 文件 | 说明 |
|---|---|
| `env_balance.py` | 非线性轮式倒立摆环境（RK4 积分，5ms 步长与固件一致） |
| `train_ppo.py` | 极简完整 PPO（GAE + 裁剪目标 + 价值损失，约 160 行） |
| `export_onnx.py` | 导出 ONNX（PC 交叉验证）+ 自动生成 `deploy/nn_weights.h` |
| `deploy/nn_controller.c` | STM32 端前向推理（3→64→64→1，约 17.5KB Flash，<0.1ms） |
| `deploy/nn_weights.h` | 由脚本生成（训完才有），**不要手改** |

## 2. 快速开始（纯 CPU 即可）

```bash
# 1) 环境（一次性）
pip install numpy matplotlib torch

# 2) 验证环境活着: 随机策略应几秒内倒下, 打印状态轨迹
python env_balance.py

# 3) 训练: CPU 约 20~40 分钟, 看到平均回报爬到 ~2800 以上即收敛
python train_ppo.py          # 产出 ppo_balance.pt + train_curve.png

# 4) 导出
pip install onnx onnxruntime  # 可选, 用于数值交叉验证
python export_onnx.py         # 产出 ppo_balance.onnx + deploy/nn_weights.h
```

**训练预期**：回报曲线应在 50~100 轮内快速上升，满回报 ≈ 2900（15s × 每步 ~1 分）。若曲线长期趴在 300 以下：检查 `env_balance.py` 里机械参数是否被改过；把 `LOG_STD0` 从 -0.5 调回 -1.0（探索小些）再试。

## 3. 固件集成（5 分钟）

1. 把 `deploy/nn_weights.h` 和 `deploy/nn_controller.c` 拷进 Keil/CubeIDE 工程源码目录；
2. 在 `main.c` 顶部加 `#include "nn_controller.c"`（或加入工程编译）；
3. 控制中断里二选一（建议用宏开关，方便论文做对照）：

```c
#define USE_RL_CONTROLLER  0        /* 0=LQR(默认), 1=RL */
...
#if USE_RL_CONTROLLER
    float u = Controller_NN_Update(Att_GetTheta(), Att_GetThetaDot()*DEG2RAD,
                                   Encoder_GetPos(), v, g_v_ref);
#else
    float u = Controller_Update(Att_GetTheta(), Att_GetThetaDot()*DEG2RAD,
                                Encoder_GetPos(), v, g_v_ref);
#endif
```

两条路径**同签名同单位**，遥测曲线可直接叠图对比——这就是论文里现成的 "LQR vs RL" 实验章节。

## 4. Sim-to-Real Checklist（逐项勾完才上实车）

| # | 项目 | 做法 | 判据 |
|---|---|---|---|
| 1 | 域随机化 | `BalanceEnv(domain_rand=True)`：质量 ±20%、力矩限幅 −30% | 随机化下训练收敛且回报 ≥2600 |
| 2 | 观测噪声注入 | 训练时给 obs 加 σ=0.01~0.05 高斯噪声（模仿卡尔曼输出残差） | 加噪后回报下降 <15% |
| 3 | 动作延迟 | 训练时动作延迟 1~2 个控制拍（模拟 I2C+PWM 链路） | 延迟 2 拍仍不发散 |
| 4 | 死区建模 | 仿真里把 |T|<死区对应值置零（与实车 DEADZONE 对齐） | 小力矩工况不抖 |
| 5 | ONNX 数值一致 | `export_onnx.py` 的交叉验证误差 <1e-5 | 通过 |
| 6 | 半实物验证 | 策略只读遥测数据离线回放，输出应与 LQR 同向 | 同向且幅值合理 |
| 7 | 台架首飞 | 车轮悬空跑策略，确认力矩方向与 LQR 一致 | 方向全对 |
| 8 | 实车接管 | 从 θ 扰动 <3° 的小任务开始，逐步放开 | T1/T2 达标 |

## 5. 常见问题

- **训练能站、实车秒倒**：九成是没做域随机化/死区建模（第 4 节 1、4 项）；剩下一成是符号——实车的"前倾为正"必须和仿真一致（跑指南第 7.3 节符号自检）。
- **策略输出一直是 0**：触发 `Controller_NN_Update` 的安全帘（|θ|>25°），说明姿态估计飘了，先修传感器再谈 RL。
- **想训转向**：把动作升到 2 维（T_left, T_right），观测加偏航角速度（MPU 的 gz），环境差速运动学加上即可——这是不错的进阶选题。
- **为什么不用 DQN**：动作是连续力矩，离散化再 DQN 精度损失大且样本效率低；连续控制标配就是 PPO/SAC/TD3。

## 6. 论文写法建议

第 6 章 "基于强化学习的控制对比实验"：PPO 训练曲线（`train_curve.png`）→ 仿真轨迹对比（LQR vs RL 对同一 3° 扰动）→ 实车 T1/T2 指标对比表 → 讨论两点：RL 无需精确模型但迁移有代价、LQR 有鲁棒裕度保证但依赖建模精度。这一章的素材本仓库已全部备齐。
