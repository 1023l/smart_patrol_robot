# -*- coding: utf-8 -*-
"""
把训练好的 SB3 PPO 模型导出为 ONNX，供 ROS2 推理节点使用。

用法示例：
    python -m inspect_rl_avoidance.export_onnx \
        --model ./models/best_model.zip --output ./rl_policy.onnx

说明：SB3 PPO 的 policy.forward(observations) 返回
(values, actions, log_probs) 三元组，无法直接导出 ONNX；
本脚本通过自定义 torch.nn.Module 包装器只输出确定性动作均值。
"""

import argparse
import os

import numpy as np
import onnxruntime as ort
import torch
import torch.nn as nn
from stable_baselines3 import PPO

# 与训练环境保持一致的观测维度：40 维激光 + 6 维辅助信息
OBS_DIM = 46


class DeterministicPolicyWrapper(nn.Module):
    """SB3 PPO 策略包装器：只输出确定性动作均值，并裁剪到动作空间范围。"""

    def __init__(self, policy):
        super().__init__()
        self.policy = policy
        # 动作边界注册为常量缓冲区，随模块一起导出
        low = torch.as_tensor(policy.action_space.low, dtype=torch.float32)
        high = torch.as_tensor(policy.action_space.high, dtype=torch.float32)
        self.register_buffer("action_low", low)
        self.register_buffer("action_high", high)

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        # 取策略分布，输出确定性动作（即分布均值，不含探索噪声）
        distribution = self.policy.get_distribution(observations)
        actions = distribution.get_actions(deterministic=True)
        return torch.clamp(actions, self.action_low, self.action_high)


def main():
    parser = argparse.ArgumentParser(description="PPO 模型导出 ONNX")
    parser.add_argument("--model", type=str, default="./models/best_model.zip",
                        help="SB3 模型 zip 路径（默认 ./models/best_model.zip）")
    parser.add_argument("--output", type=str, default="./rl_policy.onnx",
                        help="ONNX 输出路径（默认 ./rl_policy.onnx）")
    args = parser.parse_args()

    if not os.path.isfile(args.model):
        raise FileNotFoundError("找不到模型文件: {}".format(args.model))

    # 强制 CPU 推理，保证导出结果与部署环境一致
    model = PPO.load(args.model, device="cpu")
    wrapper = DeterministicPolicyWrapper(model.policy)
    wrapper.eval()

    dummy_obs = torch.zeros(1, OBS_DIM, dtype=torch.float32)
    out_dir = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(out_dir, exist_ok=True)

    torch.onnx.export(
        wrapper,
        (dummy_obs,),
        args.output,
        input_names=["obs"],
        output_names=["action"],
        opset_version=13,
        do_constant_folding=True,
    )
    print("ONNX 已导出: {}".format(args.output))

    # 用随机输入核对 PyTorch 与 ONNX 的输出一致性
    with torch.no_grad():
        obs = torch.randn(1, OBS_DIM, dtype=torch.float32)
        ref = wrapper(obs).numpy()
    session = ort.InferenceSession(args.output, providers=["CPUExecutionProvider"])
    out = session.run(None, {"obs": obs.numpy()})[0]
    max_err = float(np.max(np.abs(ref - out)))
    print("PyTorch 输出: {}".format(ref))
    print("ONNX 输出:    {}".format(out))
    print("最大误差: {:.3e}".format(max_err))
    if max_err > 1e-5:
        raise RuntimeError("ONNX 输出与 PyTorch 输出偏差过大，请检查导出配置")


if __name__ == "__main__":
    main()
