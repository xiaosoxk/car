# sim_lqr.py — LQR 闭环恢复仿真（numpy 即可，无需控制工具箱）
# 图4-1 由本脚本生成；闭环行为应与实车遥测曲线定性一致
# 运行: python sim_lqr.py   （依赖: pip install numpy matplotlib）
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

# ---- 模型参数(与 design_lqr.m 一致) ----
Mp, L, Jp, m, r, g = 1.5, 0.1, 0.005, 0.05, 0.03, 9.8
Ib = Jp + Mp * L**2
Meq = Mp + 2 * (0.5 * m * r**2 + m * r**2) / r**2
D = Ib * Meq - Mp**2 * L**2
a1 =  Meq * Mp * g * L / D;  b1 = -2 * Mp * L / (r * D)
a2 = -Mp**2 * L**2 * g / D;  b2 =  2 * Ib / (r * D)
A = np.array([[0, 1, 0, 0], [a1, 0, 0, 0], [0, 0, 0, 1], [a2, 0, 0, 0]])
B = np.array([[0], [b1], [0], [b2]])

# ---- 精确离散(ZOH): 矩阵指数级数展开 ----
def expm(M):
    X = np.eye(M.shape[0]); t = np.eye(M.shape[0])
    for k in range(1, 40):
        t = t @ M / k; X = X + t
    return X

T = 0.005
Z = np.zeros((5, 5)); Z[:4, :4] = A * T; Z[:4, 4:] = B * T
E = expm(Z); Ad, Bd = E[:4, :4], E[:4, 4]

# ---- 迭代离散 Riccati ----
def dare(A, B, Q, R, iters=50000):
    P = Q.copy()
    for _ in range(iters):
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        Pn = Q + A.T @ P @ (A - B @ K)
        if np.max(np.abs(Pn - P)) < 1e-10 * max(1.0, np.max(np.abs(P))):
            P = Pn; break
        P = Pn
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    return K

Q1 = np.diag([50.0, 2.0, 1.0, 100.0]); R = np.array([[1.0]])
Q2 = np.diag([10.0, 0.5, 1.0, 20.0])
K1 = dare(Ad, Bd, Q1, R); K2 = dare(Ad, Bd, Q2, R)
print("K(Q1) =", np.round(K1.flatten(), 3))
print("K(Q2) =", np.round(K2.flatten(), 3))
print("闭环 |eig| =", np.round(np.abs(np.linalg.eigvals(Ad - Bd @ K1)), 4))

# ---- 闭环仿真: 初始 3°, 力矩限幅 ±0.6 N·m ----
def sim(K, x0, T=3.0, umax=0.6):
    n = int(T / 0.005); X = np.zeros((n, 4)); U = np.zeros(n)
    x = np.array(x0, float)
    for k in range(n):
        u = np.clip(-(K @ x.reshape(-1, 1)).item(), -umax, umax)
        U[k] = u; X[k] = x
        x = Ad @ x + Bd.flatten() * u
    return X, U

deg = np.pi / 180
Xa, Ua = sim(K1, [3 * deg, 0, 0, 0])
Xb, Ub = sim(K2, [3 * deg, 0, 0, 0])
tt = np.arange(len(Xa)) * 0.005

fig, axs = plt.subplots(3, 1, figsize=(9.0, 7.4), dpi=200, sharex=True,
                        gridspec_kw={"height_ratios": [2, 1.6, 1.6]})
axs[0].plot(tt, Xa[:, 0] / deg, lw=2, label="Q=diag(50,2,1,100) R=1")
axs[0].plot(tt, Xb[:, 0] / deg, lw=1.6, ls="--", label="Q=diag(10,0.5,1,20)")
axs[0].set_ylabel("倾角 θ (°)"); axs[0].legend(frameon=False, fontsize=9)
axs[1].plot(tt, Xa[:, 3], lw=1.8); axs[1].set_ylabel("车速 v (m/s)")
axs[2].plot(tt, Ua, lw=1.4, color="k"); axs[2].set_ylabel("力矩 u (N·m)")
axs[2].set_xlabel("时间 (s)")
for a in axs:
    a.grid(alpha=0.3, ls=":")
    for s in ["top", "right"]: a.spines[s].set_visible(False)
axs[0].set_title("LQR 闭环恢复仿真：初始扰动 3°，力矩限幅 ±0.6 N·m")
plt.tight_layout(); plt.savefig("sim_lqr_result.png", dpi=150)
print("已保存 sim_lqr_result.png")
