# env_balance.py — 轮式倒立摆强化学习环境（与实车同构）
# 动力学: 手册 4.1 节的非线性形式（sinθ/cosθ 未线性化）
#   [Meq        MpL    ] [ẍ ]   [  2T/r        ]
#   [MpL·cosθ   Ib     ] [θ̈] = [ Mp·g·L·sinθ  ]
# 控制周期 dt=0.005s 与固件一致；动作=双轮合计力矩
# 运行: python env_balance.py   （随机策略滚动，打印每秒状态，验证环境活着）
import numpy as np

# ---- 机械参数(与 design_lqr.m / 固件一致) ----
Mp, L, Jp = 1.5, 0.10, 0.005
m, r = 0.05, 0.03
g = 9.8
Ib = Jp + Mp * L**2
Meq = Mp + 2 * (0.5 * m * r**2 + m * r**2) / r**2

T_MAX = 0.6          # 力矩限幅 N·m (与仿真/实车一致)
DT = 0.005           # 5ms 控制周期
THETA_TERM = 0.35    # |θ| > 20° 判失败
MAX_STEPS = 3000     # 15s 一个回合
OBS_SCALE = np.array([0.30, 3.0, 1.5])   # θ(rad)/θ̇(rad/s)/v(m/s) 归一化尺度


class BalanceEnv:
    """Gymnasium 风格: reset() -> obs; step(a) -> obs, r, terminated, truncated, info"""

    def __init__(self, domain_rand=False, seed=None):
        self.rng = np.random.default_rng(seed)
        self.domain_rand = domain_rand      # 域随机化: sim-to-real 必开(见 rl/README)
        self.reset()

    # ---- 非线性动力学一步(RK4) ----
    def _deriv(self, s, T):
        th, thd, x, v = s
        c = np.cos(th)
        A = np.array([[Meq, Mp * L], [Mp * L * c, Ib]])
        b = np.array([2.0 * T / r, Mp * g * L * np.sin(th)])
        acc = np.linalg.solve(A, b)         # [ẍ, θ̈]
        return np.array([thd, acc[1], v, acc[0]])

    def _dynamics_step(self, s, T):
        k1 = self._deriv(s, T)
        k2 = self._deriv(s + 0.5 * DT * k1, T)
        k3 = self._deriv(s + 0.5 * DT * k2, T)
        k4 = self._deriv(s + DT * k3, T)
        return s + DT / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

    def _obs(self):
        return (np.array([self.s[0], self.s[1], self.s[3]]) / OBS_SCALE).astype(np.float32)

    def reset(self, theta0=None):
        if self.domain_rand:
            # 域随机化: 质量/限幅/零偏扰动, 提高迁移存活率
            self.Mp_e = Mp * self.rng.uniform(0.8, 1.2)
            self.T_max_e = T_MAX * self.rng.uniform(0.7, 1.0)
        else:
            self.Mp_e, self.T_max_e = Mp, T_MAX
        th = self.rng.uniform(-0.06, 0.06) if theta0 is None else theta0
        self.s = np.array([th, 0.0, 0.0, 0.0])
        self.n = 0
        return self._obs()

    def step(self, action):
        a = float(np.clip(action, -1.0, 1.0))
        T = a * self.T_max_e
        self.s = self._dynamics_step(self.s, T)
        self.n += 1
        th, thd, x, v = self.s
        terminated = abs(th) > THETA_TERM
        truncated = self.n >= MAX_STEPS
        r = 1.0 - 5.0 * th * th - 0.05 * v * v    # 活着+姿态小+速度别野
        return self._obs(), r, terminated, truncated, {"theta": th, "v": v}


if __name__ == "__main__":
    env = BalanceEnv(seed=0)
    obs = env.reset()
    total, steps = 0.0, 0
    while steps < 1500:                      # 随机策略: 预期几秒内倒下
        obs, r, term, trunc, info = env.step(env.rng.uniform(-1, 1))
        total += r; steps += 1
        if steps % 200 == 0:
            print(f"t={steps*DT:5.2f}s  θ={info['theta']*57.3:7.2f}°  r={r:6.2f}")
        if term or trunc:
            print(f"终止于 t={steps*DT:.2f}s, 累计奖励 {total:.1f}")
            break
    print("环境自检通过: 动力学/奖励/终止条件均正常")
