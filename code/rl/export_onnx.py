# export_onnx.py — 把训练好的策略导出为 ONNX + STM32 可用的 C 权重头文件
# 依赖: pip install torch numpy onnx
# 运行: python export_onnx.py
# 产出: ppo_balance.onnx(上位机验证用) / deploy/nn_weights.h(固件直接 include)
import os
import numpy as np
import torch
from train_ppo import ActorCritic          # 复用同一网络结构

# 与 env_balance.py 保持一致（固件端 nn_weights.h 会带上这些尺度）
OBS_SCALE = np.array([0.30, 3.0, 1.5])
T_MAX = 0.6

ac = ActorCritic()
ac.load_state_dict(torch.load("ppo_balance.pt", map_location="cpu"))
ac.eval()

dummy = torch.zeros(1, 3)

# ---- 1) ONNX 导出（PC 端用 onnxruntime 交叉验证） ----
torch.onnx.export(
    ac.pi, dummy, "ppo_balance.onnx",
    input_names=["obs"], output_names=["action"],
    dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
    opset_version=13)
print("已导出 ppo_balance.onnx")

# ---- 2) 数值交叉验证: torch vs onnxruntime ----
try:
    import onnxruntime as ort
    x = np.random.randn(8, 3).astype(np.float32)
    with torch.no_grad():
        y_torch = ac.pi(torch.as_tensor(x)).numpy()
    y_ort = ort.InferenceSession("ppo_balance.onnx").run(None, {"obs": x})[0]
    err = np.max(np.abs(y_torch - y_ort))
    print(f"torch vs onnxruntime 最大误差 {err:.2e} (应 <1e-5)")
except ImportError:
    print("(未装 onnxruntime, 跳过交叉验证 — 建议安装后确认)")

# ---- 3) 生成固件头文件 deploy/nn_weights.h ----
sd = ac.state_dict()
w1 = sd["pi.0.weight"].numpy();  b1 = sd["pi.0.bias"].numpy()
w2 = sd["pi.2.weight"].numpy();  b2 = sd["pi.2.bias"].numpy()
w3 = sd["pi.4.weight"].numpy();  b3 = sd["pi.4.bias"].numpy()

def arr(name, a):
    s = a.astype(np.float32).reshape(-1)
    body = ",\n    ".join(", ".join(f"{v:.6e}f" for v in s[i:i+4])
                          for i in range(0, len(s), 4))
    return f"static const float {name}[{len(s)}] = {{\n    {body}\n}};\n"

hdr = f"""/**
 * nn_weights.h — PPO 策略网络权重(由 export_onnx.py 自动生成, 勿手改)
 * 网络: 3 -> 64(tanh) -> 64(tanh) -> 1(线性)   共 {w1.size+b1.size+w2.size+b2.size+w3.size+b3.size} 个参数 ≈ {(w1.size+b1.size+w2.size+b2.size+w3.size+b3.size)*4/1024:.1f} KB Flash
 * 观测归一化: obs = [θ(rad)/{OBS_SCALE[0]}, θ̇(rad/s)/{OBS_SCALE[1]}, v(m/s)/{OBS_SCALE[2]}]
 * 输出: a ∈ [-1,1] -> 力矩 u = a × {T_MAX} N·m
 */
#ifndef NN_WEIGHTS_H
#define NN_WEIGHTS_H

#define NN_OBS_TH_SCALE   {OBS_SCALE[0]}f
#define NN_OBS_THD_SCALE  {OBS_SCALE[1]}f
#define NN_OBS_V_SCALE    {OBS_SCALE[2]}f
#define NN_T_MAX          {T_MAX}f

{arr("NN_W1", w1)}{arr("NN_B1", b1)}{arr("NN_W2", w2)}{arr("NN_B2", b2)}{arr("NN_W3", w3)}{arr("NN_B3", b3)}
#endif /* NN_WEIGHTS_H */
"""
os.makedirs("deploy", exist_ok=True)
with open("deploy/nn_weights.h", "w", encoding="utf-8") as f:
    f.write(hdr)
print("已生成 deploy/nn_weights.h —— 拷进固件工程, 配合 deploy/nn_controller.c 使用")
