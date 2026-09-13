本目录用于存放 RL 避障节点推理所需的策略模型文件 rl_policy.onnx。

模型来源（在训练机上执行）：
1. 训练 PPO 策略（约 50 万步，训练过程会自动保存评估最优模型 best_model.zip）：
   python -m inspect_rl_avoidance.train_ppo --timesteps 500000 --save-dir ./models
2. 导出 ONNX（输入 shape 为 (1, 46)，opset 13，输出确定性动作均值）：
   python -m inspect_rl_avoidance.export_onnx --model ./models/best_model.zip --output ./rl_policy.onnx
3. 将导出的 rl_policy.onnx 复制到本目录，然后在 ROS2 工作空间重新编译：
   colcon build --packages-select inspect_rl_avoidance

注意：缺少 rl_policy.onnx 时 rl_avoidance 节点会启动失败；
也可通过 launch 参数 onnx_path 指定其他路径的模型文件。

Python 依赖（训练/导出/推理，通过 pip 安装）：
    pip install gymnasium pybullet stable-baselines3 torch onnx onnxruntime
