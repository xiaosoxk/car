# train_ppo.py — 极简完整 PPO，训练倒立摆平衡策略
# 依赖: pip install torch numpy matplotlib
# 运行: python train_ppo.py            # CPU 约 20~40 分钟收敛
# 产出: ppo_balance.pt(最优权重) / train_curve.png(奖励曲线)
import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from env_balance import BalanceEnv, MAX_STEPS

torch.manual_seed(0); np.random.seed(0)
DEV = "cuda" if torch.cuda.is_available() else "cpu"

# ---- 超参数 ----
HID = 64
LR = 3e-4
GAMMA, LAM = 0.999, 0.95
CLIP = 0.2
EPOCHS = 10
MB_SIZE = 256
STEPS = 4096              # 每次更新收集的步数
ITERS = 400               # 总更新轮数
LOG_STD0 = -0.5           # 初始探索噪声

def mlp(sizes, act=nn.Tanh):
    layers = []
    for i in range(len(sizes) - 2):
        layers += [nn.Linear(sizes[i], sizes[i + 1]), act()]
    layers += [nn.Linear(sizes[-2], sizes[-1])]
    return nn.Sequential(*layers)

class ActorCritic(nn.Module):
    def __init__(self, obs_dim=3, act_dim=1):
        super().__init__()
        self.pi = mlp([obs_dim, HID, HID, act_dim])
        self.v  = mlp([obs_dim, HID, HID, 1])
        self.log_std = nn.Parameter(torch.ones(act_dim) * LOG_STD0)

    def dist(self, obs):
        return Normal(self.pi(obs), self.log_std.exp())

    def act(self, obs):
        with torch.no_grad():
            d = self.dist(obs)
            a = d.sample()
            return a, d.log_prob(a).sum(-1), self.v(obs).squeeze(-1)

ac = ActorCritic().to(DEV)
opt = torch.optim.Adam(ac.parameters(), lr=LR)
env = BalanceEnv(seed=1)

def gae(rewards, values, dones, last_v):
    adv = np.zeros(len(rewards), dtype=np.float32)
    gae_t = 0.0
    for t in reversed(range(len(rewards))):
        nxt = last_v if t == len(rewards) - 1 else values[t + 1]
        nxt = 0.0 if dones[t] else nxt
        delta = rewards[t] + GAMMA * nxt - values[t]
        gae_t = delta + GAMMA * LAM * (0.0 if dones[t] else gae_t)
        adv[t] = gae_t
    return adv

obs = env.reset()
ep_ret, ep_len, ep_returns = 0.0, 0, []
best = -1e9
history = []

for it in range(1, ITERS + 1):
    buf = {k: [] for k in ("obs", "act", "logp", "val", "rew", "done")}
    for _ in range(STEPS):
        o = torch.as_tensor(obs, dtype=torch.float32, device=DEV)
        a, logp, v = ac.act(o)
        nobs, r, term, trunc, info = env.step(a.cpu().numpy()[0])
        for k, val in zip(("obs", "act", "logp", "val", "rew", "done"),
                          (obs, a.cpu().numpy(), logp.cpu().numpy(),
                           v.cpu().item(), r, term or trunc)):
            buf[k].append(val)
        ep_ret += r; ep_len += 1
        obs = nobs
        if term or trunc:
            ep_returns.append(ep_ret); ep_ret, ep_len = 0.0, 0
            obs = env.reset()

    O = torch.as_tensor(np.array(buf["obs"]), dtype=torch.float32, device=DEV)
    A = torch.as_tensor(buf["act"], dtype=torch.float32, device=DEV)
    with torch.no_grad():
        V = ac.v(O).squeeze(-1).cpu().numpy()
    adv = gae(buf["rew"], V, buf["done"], V[-1])
    adv_t = torch.as_tensor((adv - adv.mean()) / (adv.std() + 1e-8),
                            dtype=torch.float32, device=DEV)
    ret_t = torch.as_tensor(adv + V, dtype=torch.float32, device=DEV)
    logp_old = torch.as_tensor(buf["logp"], dtype=torch.float32, device=DEV)

    idx = np.arange(STEPS)
    for _ in range(EPOCHS):
        np.random.shuffle(idx)
        for s0 in range(0, STEPS, MB_SIZE):
            mb = torch.as_tensor(idx[s0:s0 + MB_SIZE], device=DEV)
            d = ac.dist(O[mb])
            logp = d.log_prob(A[mb]).sum(-1)
            ratio = (logp - logp_old[mb]).exp()
            pi_loss = -torch.min(ratio * adv_t[mb],
                                 torch.clamp(ratio, 1 - CLIP, 1 + CLIP) * adv_t[mb]).mean()
            v_loss = ((ac.v(O[mb]).squeeze(-1) - ret_t[mb]) ** 2).mean()
            opt.zero_grad()
            (pi_loss + 0.5 * v_loss).backward()
            nn.utils.clip_grad_norm_(ac.parameters(), 0.5)
            opt.step()

    mean_ret = np.mean(ep_returns[-20:]) if ep_returns else 0.0
    history.append(mean_ret)
    if it % 10 == 0:
        print(f"iter {it:3d}/{ITERS}  平均回报(最近20回合) {mean_ret:8.1f}  "
              f"log_std {ac.log_std.item():.2f}")
    if mean_ret > best and len(ep_returns) >= 5:
        best = mean_ret
        torch.save(ac.state_dict(), "ppo_balance.pt")

# ---- 曲线 + 最终保存 ----
plt.figure(figsize=(7, 4))
plt.plot(history)
plt.xlabel("PPO 更新轮"); plt.ylabel("平均回合回报")
plt.title("PPO 训练曲线（越高越好，满回报≈2900）")
plt.grid(alpha=0.3); plt.tight_layout()
plt.savefig("train_curve.png", dpi=150)
torch.save(ac.state_dict(), "ppo_balance_final.pt")
print(f"完成: 最优权重 ppo_balance.pt (回报 {best:.0f}), 曲线 train_curve.png")
