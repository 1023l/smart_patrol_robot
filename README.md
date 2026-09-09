# 工业巡检机器人导航系统（inspect_ws）

面向 5 层工业厂房全自动巡检场景，基于 ROS2 Humble + Nav2 的轮式机器人
导航 + 到点视觉检测系统。针对厂房玻璃墙体激光穿透、窄通道易卡死、人流动态干扰、
跨电梯多楼层定位漂移四大量产痛点，对 Nav2 框架做多模块 C++ 深度二次开发
与工程落地优化。

## 功能包总览

| 功能包 | 语言 | 对应能力 |
|---|---|---|
| `nav2_gradient_planner` | C++ | 梯度感知自定义 A* 全局规划器插件（连续梯度代价，路径主动远离高危区；目标贴墙容差搜索） |
| `nav2_keepout_layer` | C++ | KeepOutZone 禁区代价地图层插件（YAML 静态多边形 + 话题动态增删，射线法栅格-禁区碰撞检测，补齐玻璃穿透虚拟障碍） |
| `inspect_speed_adapter` | C++ | 七扇区自适应调速（Nav2 speed_limit 标准接口作用于 TEB）+ 导航指标观测（规划时延/任务完成率/路径平滑度/冻结检测）+ TEB 终点震荡调参记录 |
| `inspect_navigation` | 配置 | Nav2 全量参数集成 + 四级递进脱困行为树 + 5 层厂房地图生成器 + 启动文件 |
| `inspect_floor_manager` | Python | 多楼层切换（load_map 服务）+ 电梯口 AMCL 重定位 + 跨层巡检任务编排（到点后调用视觉） |
| `inspect_vision` | Python | 到点视觉巡检（抓图/合成图 → OpenCV 规则或可选 YOLO → 结果落盘） |
| `inspect_rl_avoidance` | Python | PPO 强化学习动态避障（PyBullet/Gymnasium 训练 → ONNX 推理）+ 代价地图安全盾 + 速度选择器 |
| `inspect_interfaces` | msg/srv | KeepoutZone / InspectResult 消息 + SwitchMap / InspectPoint 服务 |

## 架构数据流

```
                                /scan ──────────────┐
                                                    ▼
用户目标(NavigateToPose)                七扇区环境分类(OPEN/CORRIDOR/CORNER/DENSE)
      │                                            │ speed_limit(百分比)
      ▼                                            ▼
bt_navigator(四级递进脱困BT) ──► planner_server ──► controller_server(TEB)
      │                              │                    │ /cmd_vel_nav
      │                        梯度感知A*                ▼
      │                     (cost_weight连续代价)   冻结检测 ──► RL接管 ──► 安全盾校验
      │                                                    │              │
      ▼                                                    ▼              ▼
KeepoutZone禁区层 ◄── keepout_zones话题            /cmd_vel_rl ──► velocity_mux ──► /cmd_vel
（玻璃穿透虚拟障碍补齐，                                    (rl_active 心搏超时回退传统控制器)
  静态YAML + 动态增删）

跨楼层巡检：
  patrol_mission ──► /switch_floor ──► floor_manager ──► load_map + /initialpose
                 ──► NavigateToPose 到点
                 ──► /inspect_point ──► inspect_vision（检测 + /tmp/inspect_results 落盘）
```

## 构建与运行

```bash
# 0. 前置：Ubuntu 22.04 + ROS2 Humble + Nav2 + Gazebo Classic
#    推荐：WSL2 Ubuntu-22.04（本仓库按 Humble 编写）
#    仿真机器人模型复用鱼香ROS教程 fishbot_description（差速底盘+360°雷达）

# 1. 生成 5 层厂房地图（需要 numpy）
cd src/inspect_navigation
python3 scripts/generate_maps.py --output-dir maps/

# 2. 编译
cd ~/inspect_ws
colcon build --symlink-install
source install/setup.bash

# 3. 启动仿真机器人
ros2 launch inspect_navigation gazebo_sim.launch.py

# 4. 启动导航系统（含规划器/keepout层/TEB/四级脱困BT/七扇区调速/指标/楼层管理）
ros2 launch inspect_navigation inspect_navigation.launch.py floor:=1

# 5. 启动到点视觉巡检（无相机时自动用合成图，保证闭环可演示）
ros2 launch inspect_vision inspect_vision.launch.py

# 6. （可选）启动 RL 混合避障（需先训练并导出 ONNX 模型到 config/）
ros2 launch inspect_rl_avoidance rl_avoidance.launch.py

# 7. 启动跨楼层巡检任务（导航到点后自动视觉检测）
ros2 launch inspect_floor_manager patrol_mission.launch.py
```

### 仅验证视觉（不依赖 ROS / 可在 Windows 上跑）

```bash
cd src/inspect_vision
python -m inspect_vision.demo_offline --out-dir ./inspect_results_demo
```

## RL 避障模型训练（可选）

```bash
pip install gymnasium pybullet stable-baselines3 torch onnx onnxruntime

# PyBullet 厂房+动态行人环境训练 PPO（约50万步）
python3 -m inspect_rl_avoidance.train_ppo --timesteps 500000 --save-dir ./models

# 导出确定性策略为 ONNX（输入46维：40激光+6辅助，输出2维：v,w）
python3 -m inspect_rl_avoidance.export_onnx --model ./models/best_model.zip \
    --output src/inspect_rl_avoidance/config/rl_policy.onnx
```

## 关键设计决策（面试索引）

1. **连续梯度代价 A\***：`traversal = dist × (1 + cost_weight × cost/253)`，
   复用 InflationLayer 输出做连续代价，不需要自建距离场；octile 启发式
   可采纳；目标贴墙时 tolerance 螺旋搜索最近可达点。
2. **keepout 层两级检测**：updateBounds 用 AABB 并集圈定范围，updateCosts
   内层"先 AABB 粗筛再射线法精判"，复杂度从 O(cells×V) 降到
   O(affected_cells×V)；互斥锁保护话题回调与代价更新并发。
3. **调速走 speed_limit 标准接口**：不侵入 TEB 控制器，换 MPPI/DWB
   零改动；分类确认机制（连续3帧一致）+ 限速斜率控制 + 断流看门狗三重防抖。
4. **四级递进脱困嵌套 RecoveryNode**：清图→后退→旋转→等待，恢复成本
   单调递增；后退 0.30m/旋转 90° 按 1.2m 通道宽+0.44m 机器人直径标定。
5. **RL 安全盾**：RL 输出前向投影 1.5s 圆弧轨迹逐点查栅格代价，任一点
   超阈值即判 unsafe 并回退传统控制器；velocity_mux 心跳超时 0.5s 兜底。
6. **终点震荡三根因**：acc_lim_x 低于实机减速（极限环）→ 0.5 匹配实机；
   min_obstacle_dist 与膨胀层冲突（贴墙目标）→ 0.35；weight_acc_lim_x
   过低（减速不坚决）→ 1.5。验证指标：nav_metrics 的 freeze_max_s 与
   path_smoothness 前后对比。
7. **到点视觉闭环**：导航技能与检测技能解耦；`/inspect_point` 服务统一入口，
   无相机时合成图兜底，有 YOLO 权重则升级，默认 OpenCV 规则可演示。
