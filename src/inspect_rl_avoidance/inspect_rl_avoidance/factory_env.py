# -*- coding: utf-8 -*-
"""
工厂巡检机器人动态避障训练环境（Gymnasium + PyBullet 后端）。

场景：20m x 20m 封闭厂房（四面静态墙），每回合随机生成 5~8 个动态行人
（半径 0.3m 球体，随机游走模型）；差速驱动机器人（半径 0.2m 扁圆柱），
通过 p.resetBaseVelocity 设置速度，由物理引擎积分位姿。

观测（46 维）= 40 维激光（机器人中心 360 度均匀采样，最大量程 6m，
归一化 0~1，射线高度避开机器人本体）+ 6 维辅助
[sin(相对目标方位角), cos(相对目标方位角), 目标距离/10, 当前线速度/0.5,
 当前角速度/1.2, 朝向与目标夹角]。
动作（2 维）= [线速度 v, 角速度 w]，v 属于 [-0.3, 0.5]，w 属于 [-1.2, 1.2]。

奖励：目标接近量*10 - 最近行人过近惩罚(<0.6m 线性) - 动作突变惩罚 - 每步-0.01；
碰撞 -100 并终止；到达目标(<0.3m) +100 并终止；500 步截断。
"""

import math
from typing import Optional, Tuple

import gymnasium as gym
import numpy as np
import pybullet as p
from gymnasium import spaces


class FactoryAvoidanceEnv(gym.Env):
    """PPO 训练用：厂房动态行人避障环境。"""

    metadata = {"render_modes": ["human"], "render_fps": 20}

    # 场景 / 机器人 / 行人常量
    WALL_HALF = 10.0          # 厂房半边长（整体 20m x 20m）
    WALL_HEIGHT = 2.0         # 墙壁高度（确保激光射线可命中）
    WALL_THICKNESS = 0.2      # 墙壁厚度
    ROBOT_RADIUS = 0.2        # 机器人半径
    ROBOT_HEIGHT = 0.1        # 机器人（圆柱）高度，做扁以避开自身激光
    PED_RADIUS = 0.3          # 行人半径
    PED_NUM_MIN = 5           # 行人数量下限
    PED_NUM_MAX = 8           # 行人数量上限
    PED_SPEED_MIN = 0.5       # 行人速度下限（m/s）
    PED_SPEED_MAX = 1.2       # 行人速度上限（m/s）
    PED_TURN_PROB = 0.08      # 行人每步随机转向概率（随机游走模型）
    # 激光常量
    NUM_RAYS = 40             # 射线条数
    LASER_RANGE = 6.0         # 最大量程（m）
    LASER_HEIGHT = 0.35       # 射线高度：高于机器人顶面 0.1m，低于行人球顶 0.6m
    # 动作空间边界
    V_MIN, V_MAX = -0.3, 0.5
    W_MIN, W_MAX = -1.2, 1.2
    # 终止与奖励常量
    GOAL_THRESHOLD = 0.3      # 到达目标判定距离（m）
    MAX_STEPS = 500           # 单回合最大步数（超过截断）
    PED_DANGER_DIST = 0.6     # 行人危险惩罚起始距离（m）
    REWARD_APPROACH = 10.0    # 目标接近奖励系数
    REWARD_PED_DANGER = 2.0   # 行人过近线性惩罚系数
    REWARD_ACTION_JUMP = 0.5  # 动作突变惩罚系数
    REWARD_STEP = -0.01       # 每步惩罚
    REWARD_COLLISION = -100.0 # 碰撞惩罚
    REWARD_GOAL = 100.0       # 到达目标奖励

    def __init__(self, render_mode: Optional[str] = None):
        super().__init__()
        if render_mode not in (None, "human"):
            raise ValueError("不支持的 render_mode: {}".format(render_mode))
        self.render_mode = render_mode

        # 初始化 PyBullet：默认无界面（DIRECT），render_mode="human" 时带界面（GUI）
        self.client = p.connect(p.GUI if render_mode == "human" else p.DIRECT)
        p.setTimeStep(1.0 / self.metadata["render_fps"], physicsClientId=self.client)
        p.setGravity(0.0, 0.0, -9.81, physicsClientId=self.client)

        # 观测空间：40 维激光 + 6 维辅助；动作空间：[线速度 v, 角速度 w]
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(46,), dtype=np.float32)
        self.action_space = spaces.Box(
            low=np.array([self.V_MIN, self.W_MIN], dtype=np.float32),
            high=np.array([self.V_MAX, self.W_MAX], dtype=np.float32),
            dtype=np.float32)

        # 静态场景：地面 + 四面墙（厚 0.2m，围出 20m x 20m 厂房）
        plane = p.createCollisionShape(p.GEOM_PLANE, physicsClientId=self.client)
        p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=plane,
                          physicsClientId=self.client)
        t = self.WALL_THICKNESS / 2.0
        far = self.WALL_HALF + t
        wh = self.WALL_HEIGHT / 2.0
        for center, ext in (((0.0, far), (far, t)), ((0.0, -far), (far, t)),
                            ((far, 0.0), (t, far)), ((-far, 0.0), (t, far))):
            col = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[ext[0], ext[1], wh],
                physicsClientId=self.client)
            p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=col,
                              basePosition=[center[0], center[1], wh],
                              physicsClientId=self.client)

        # 机器人（扁圆柱）：摩擦与阻尼置零，保证 resetBaseVelocity
        # 设置的速度在一个物理步内不被削减
        body = p.createCollisionShape(
            p.GEOM_CYLINDER, radius=self.ROBOT_RADIUS,
            height=self.ROBOT_HEIGHT, physicsClientId=self.client)
        self.robot_id = p.createMultiBody(
            baseMass=1.0, baseCollisionShapeIndex=body,
            basePosition=[0.0, 0.0, self.ROBOT_HEIGHT / 2.0],
            physicsClientId=self.client)
        p.changeDynamics(self.robot_id, -1, lateralFriction=0.0,
                         linearDamping=0.0, angularDamping=0.0,
                         physicsClientId=self.client)

        # 行人碰撞形状（球体，半径 0.3m），实体在 reset 时按随机数量创建
        self._ped_col = p.createCollisionShape(
            p.GEOM_SPHERE, radius=self.PED_RADIUS, physicsClientId=self.client)

        # 运行时状态
        self._robot_xy = np.zeros(2, dtype=np.float64)
        self._robot_yaw = 0.0
        self._cur_v = 0.0
        self._cur_w = 0.0
        self._prev_action = np.zeros(2, dtype=np.float64)
        self._goal_xy = np.zeros(2, dtype=np.float64)
        self._prev_goal_dist = 0.0
        self._ped_ids = []
        self._ped_speeds = []
        self._ped_headings = []
        self._step_count = 0

    def reset(self, seed: Optional[int] = None,
              options: Optional[dict] = None) -> Tuple[np.ndarray, dict]:
        super().reset(seed=seed)
        # 清理上一回合的行人实体
        for ped_id in self._ped_ids:
            p.removeBody(ped_id, physicsClientId=self.client)
        self._ped_ids, self._ped_speeds, self._ped_headings = [], [], []

        # 随机起点与目标（二者相距 8~14m 且均离墙至少 1m）
        start, goal = self._sample_start_goal()
        self._goal_xy = goal
        self._robot_xy = start
        self._robot_yaw = float(self.np_random.uniform(-math.pi, math.pi))
        p.resetBasePositionAndOrientation(
            self.robot_id, [start[0], start[1], self.ROBOT_HEIGHT / 2.0],
            p.getQuaternionFromEuler([0.0, 0.0, self._robot_yaw]),
            physicsClientId=self.client)
        p.resetBaseVelocity(self.robot_id, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                            physicsClientId=self.client)

        # 随机生成 5~8 个行人，并重置回合状态
        self._spawn_pedestrians()
        self._cur_v, self._cur_w = 0.0, 0.0
        self._prev_action = np.zeros(2, dtype=np.float64)
        self._prev_goal_dist = float(np.linalg.norm(goal - start))
        self._step_count = 0
        obs = self._build_observation()
        return obs, {"goal_dist": self._prev_goal_dist}

    def step(self, action) -> Tuple[np.ndarray, float, bool, bool, dict]:
        self._step_count += 1
        action = np.clip(np.asarray(action, dtype=np.float64).reshape(-1),
                         [self.V_MIN, self.W_MIN], [self.V_MAX, self.W_MAX])
        # 动作突变惩罚：相邻两步动作差的平方和
        action_jump = float(np.sum(np.square(action - self._prev_action)))
        self._prev_action = action
        v, w = float(action[0]), float(action[1])
        self._cur_v, self._cur_w = v, w

        # 差速运动学：把 (v, w) 写入基座速度，由物理引擎积分位姿
        yaw = self._robot_yaw
        p.resetBaseVelocity(self.robot_id,
                            [v * math.cos(yaw), v * math.sin(yaw), 0.0],
                            [0.0, 0.0, w], physicsClientId=self.client)
        # 行人随机游走
        self._update_pedestrians()
        p.stepSimulation(physicsClientId=self.client)
        self._sync_robot_state()

        # 奖励：目标接近量*10 - 最近行人过近惩罚 - 动作突变惩罚 - 每步惩罚
        goal_vec = self._goal_xy - self._robot_xy
        goal_dist = float(np.linalg.norm(goal_vec))
        reward = self.REWARD_APPROACH * (self._prev_goal_dist - goal_dist)
        min_ped_dist = self._min_pedestrian_distance()
        if min_ped_dist < self.PED_DANGER_DIST:
            reward -= self.REWARD_PED_DANGER * (self.PED_DANGER_DIST - min_ped_dist)
        reward -= self.REWARD_ACTION_JUMP * action_jump
        reward += self.REWARD_STEP

        # 终止判断：碰撞 / 到达目标 / 步数截断
        terminated = truncated = False
        collision = self._check_collision()
        if collision:
            reward += self.REWARD_COLLISION
            terminated = True
        elif goal_dist < self.GOAL_THRESHOLD:
            reward += self.REWARD_GOAL
            terminated = True
        elif self._step_count >= self.MAX_STEPS:
            truncated = True
        self._prev_goal_dist = goal_dist

        obs = self._build_observation()
        info = {"goal_dist": goal_dist, "min_ped_dist": min_ped_dist,
                "collision": collision,
                "success": (not collision) and goal_dist < self.GOAL_THRESHOLD}
        return obs, float(reward), terminated, truncated, info

    def render(self):
        # 仅 human 模式下调整视角，俯视整个厂房
        if self.render_mode != "human":
            return
        p.resetDebugVisualizerCamera(
            cameraDistance=24.0, cameraYaw=45.0, cameraPitch=-89.0,
            cameraTargetPosition=[0.0, 0.0, 0.0], physicsClientId=self.client)

    def close(self):
        if p.isConnected(self.client):
            p.disconnect(self.client)

    def _sample_start_goal(self) -> Tuple[np.ndarray, np.ndarray]:
        """随机采样起点与目标：均距墙至少 1m，二者相距 8~14m。"""
        margin = 1.0
        lo, hi = -self.WALL_HALF + margin, self.WALL_HALF - margin
        while True:
            start = self.np_random.uniform(lo, hi, size=2)
            theta = self.np_random.uniform(0.0, 2.0 * math.pi)
            dist = self.np_random.uniform(8.0, 14.0)
            goal = start + dist * np.array([math.cos(theta), math.sin(theta)])
            if np.all(goal > lo) and np.all(goal < hi):
                return start.copy(), goal.copy()

    def _spawn_pedestrians(self):
        """随机生成 5~8 个行人，位置避开起点周围 3m 与目标周围 2m。"""
        num = int(self.np_random.integers(self.PED_NUM_MIN, self.PED_NUM_MAX + 1))
        lo, hi = -self.WALL_HALF + 0.5, self.WALL_HALF - 0.5
        placed = []
        for _ in range(num):
            pos = None
            for _ in range(200):  # 拒绝采样：远离起点/目标/其他行人
                cand = self.np_random.uniform(lo, hi, size=2)
                if np.linalg.norm(cand - self._robot_xy) < 3.0:
                    continue
                if np.linalg.norm(cand - self._goal_xy) < 2.0:
                    continue
                if any(np.linalg.norm(cand - q) < 1.0 for q in placed):
                    continue
                pos = cand
                break
            if pos is None:
                continue  # 多次尝试仍未找到合法位置则放弃该行人
            placed.append(pos)
            ped_id = p.createMultiBody(
                baseMass=1.0, baseCollisionShapeIndex=self._ped_col,
                basePosition=[pos[0], pos[1], self.PED_RADIUS],
                physicsClientId=self.client)
            # 摩擦置零，行人速度完全由随机游走模型控制
            p.changeDynamics(ped_id, -1, lateralFriction=0.0,
                             linearDamping=0.0, angularDamping=0.0,
                             physicsClientId=self.client)
            self._ped_ids.append(ped_id)
            self._ped_speeds.append(float(
                self.np_random.uniform(self.PED_SPEED_MIN, self.PED_SPEED_MAX)))
            self._ped_headings.append(float(
                self.np_random.uniform(-math.pi, math.pi)))

    def _update_pedestrians(self):
        """随机游走：每步以固定概率转向，接近墙壁时把朝向拨回厂房内部。"""
        limit = self.WALL_HALF - self.PED_RADIUS - 0.2
        for idx, ped_id in enumerate(self._ped_ids):
            if self.np_random.random() < self.PED_TURN_PROB:
                self._ped_headings[idx] += self.np_random.uniform(
                    -math.pi / 2.0, math.pi / 2.0)
            pos, _ = p.getBasePositionAndOrientation(ped_id, physicsClientId=self.client)
            x, y = pos[0], pos[1]
            if abs(x) > limit or abs(y) > limit:
                self._ped_headings[idx] = math.atan2(-y, -x) + \
                    self.np_random.uniform(-0.5, 0.5)
            speed, heading = self._ped_speeds[idx], self._ped_headings[idx]
            p.resetBaseVelocity(ped_id,
                                [speed * math.cos(heading),
                                 speed * math.sin(heading), 0.0],
                                [0.0, 0.0, 0.0], physicsClientId=self.client)

    def _sync_robot_state(self):
        """从物理引擎读取机器人最新位姿。"""
        pos, quat = p.getBasePositionAndOrientation(self.robot_id, physicsClientId=self.client)
        euler = p.getEulerFromQuaternion(quat)
        self._robot_xy = np.array([pos[0], pos[1]], dtype=np.float64)
        self._robot_yaw = float(euler[2])

    def _min_pedestrian_distance(self) -> float:
        """机器人中心到最近行人中心的距离。"""
        best = float("inf")
        for ped_id in self._ped_ids:
            pos, _ = p.getBasePositionAndOrientation(ped_id, physicsClientId=self.client)
            dist = float(np.linalg.norm(np.array(pos[:2]) - self._robot_xy))
            best = min(best, dist)
        return best

    def _check_collision(self) -> bool:
        """通过接触点检测机器人与任何物体（行人/墙壁）的碰撞。"""
        contacts = p.getContactPoints(bodyA=self.robot_id, physicsClientId=self.client)
        return len(contacts) > 0

    def _build_observation(self) -> np.ndarray:
        """构造 46 维观测：40 维激光 + 6 维辅助信息。"""
        # 40 维激光：以机器人朝向为 0 度，从 -pi 到 +pi 均匀采样，
        # 射线高度 LASER_HEIGHT 避开机器人本体，只命中行人与墙壁
        angles = self._robot_yaw - math.pi + \
            (np.arange(self.NUM_RAYS, dtype=np.float64) + 0.5) * \
            (2.0 * math.pi / self.NUM_RAYS)
        x, y, z = self._robot_xy[0], self._robot_xy[1], self.LASER_HEIGHT
        ray_from = [[x, y, z]] * self.NUM_RAYS
        ray_to = [[x + self.LASER_RANGE * math.cos(a),
                   y + self.LASER_RANGE * math.sin(a), z] for a in angles]
        results = p.rayTestBatch(ray_from, ray_to, physicsClientId=self.client)
        # hit_fraction 属于 [0,1]：命中时为实际距离/射线长度，未命中为 1.0，
        # 与"按最大量程 6m 归一化到 0~1"完全等价
        laser = np.clip(np.array([r[2] for r in results], dtype=np.float32), 0.0, 1.0)

        # 6 维辅助：[sin(相对目标方位角), cos(相对目标方位角), 目标距离/10,
        # 当前线速度/0.5, 当前角速度/1.2, 朝向与目标夹角]
        goal_vec = self._goal_xy - self._robot_xy
        goal_dist = float(np.linalg.norm(goal_vec))
        bearing = math.atan2(goal_vec[1], goal_vec[0]) - self._robot_yaw
        bearing = (bearing + math.pi) % (2.0 * math.pi) - math.pi  # 归一化到 [-pi, pi]
        aux = np.array([math.sin(bearing), math.cos(bearing), goal_dist / 10.0,
                        self._cur_v / 0.5, self._cur_w / 1.2, bearing],
                       dtype=np.float32)
        return np.concatenate([laser, aux]).astype(np.float32)
