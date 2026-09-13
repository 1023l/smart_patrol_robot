# -*- coding: utf-8 -*-
"""
PPO 训练入口：在厂房动态行人避障环境中训练"激光观测 -> 差速速度"策略。

用法示例：
    python -m inspect_rl_avoidance.train_ppo --timesteps 500000 --save-dir ./models

训练结束后：
- best_model.zip ：评估最优模型（EvalCallback 自动保存），推荐用于导出 ONNX；
- ppo_final.zip  ：训练结束时的最终模型。
"""

import argparse
import os

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback

from inspect_rl_avoidance.factory_env import FactoryAvoidanceEnv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PPO 厂房避障策略训练")
    parser.add_argument("--timesteps", type=int, default=500000,
                        help="总训练步数（默认 500000）")
    parser.add_argument("--save-dir", type=str, default="./models",
                        help="模型与日志保存目录（默认 ./models）")
    return parser.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.save_dir, exist_ok=True)

    # 训练环境与独立评估环境（均为随机场景，避免评估泄漏训练轨迹）
    env = FactoryAvoidanceEnv()
    eval_env = FactoryAvoidanceEnv()

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        policy_kwargs=dict(net_arch=[128, 128]),
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=256,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.005,
        tensorboard_log=os.path.join(args.save_dir, "tensorboard"),
    )

    # 每 20000 步评估一次，自动保存最优模型 best_model.zip
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=args.save_dir,
        log_path=args.save_dir,
        eval_freq=20000,
        n_eval_episodes=10,
        deterministic=True,
    )

    model.learn(total_timesteps=args.timesteps, callback=eval_callback)

    # 保存最终模型
    final_path = os.path.join(args.save_dir, "ppo_final")
    model.save(final_path)
    print("训练完成，最终模型已保存至 {}".format(final_path + ".zip"))

    env.close()
    eval_env.close()


if __name__ == "__main__":
    main()
