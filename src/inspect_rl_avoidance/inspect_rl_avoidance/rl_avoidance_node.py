# -*- coding: utf-8 -*-
"""
RL 避障节点：ONNX 策略推理 + 代价地图安全盾。

1. 订阅 /cmd_vel_nav 做冻结检测：|v|<0.03 且 |w|<0.03 持续 freeze_timeout 秒
   且目标距离大于 goal_min_dist 时 RL 接管；
2. 接管期间构造与训练一致的 46 维观测，onnxruntime 推理候选速度 (v, w)；
3. 安全盾：候选速度沿圆弧轨迹前向投影 shield_horizon 秒（步长 shield_dt），
   任一投影点代价 data[j*width+i] > occupancy_threshold 或越界则判定不安全，
   发布零速并回退传统控制器；安全则发布到 /cmd_vel_rl 且 /rl_active 为 True；
4. 目标距离小于 goal_min_dist 或 /cmd_vel_nav 恢复正常输出时退出接管，
   最终速度切换由 velocity_mux 节点完成。
"""

import math

import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray

# 与训练环境保持一致的观测/激光/动作参数
NUM_RAYS, LASER_RANGE, OBS_DIM = 40, 6.0, 46
V_MIN, V_MAX, W_MIN, W_MAX = -0.3, 0.5, -1.2, 1.2
# 冻结判定阈值：线速度与角速度绝对值均低于该值视为"零速"
FREEZE_VEL_EPS = 0.03


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """四元数转偏航角。"""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


class RLAvoidanceNode(Node):
    """PPO 策略推理节点，输出必须经过代价地图安全盾校验。"""

    def __init__(self):
        super().__init__("rl_avoidance")

        # ---- 参数 ----
        for name, value in (("onnx_path", ""), ("freeze_timeout", 3.0),
                            ("shield_horizon", 1.5), ("shield_dt", 0.1),
                            ("occupancy_threshold", 65), ("goal_min_dist", 1.0)):
            self.declare_parameter(name, value)
        gp = self.get_parameter
        onnx_path = gp("onnx_path").get_parameter_value().string_value
        self.freeze_timeout = gp("freeze_timeout").get_parameter_value().double_value
        self.shield_horizon = gp("shield_horizon").get_parameter_value().double_value
        self.shield_dt = gp("shield_dt").get_parameter_value().double_value
        self.occupancy_threshold = gp("occupancy_threshold").get_parameter_value().integer_value
        self.goal_min_dist = gp("goal_min_dist").get_parameter_value().double_value
        if not onnx_path:
            self.get_logger().error("必须通过参数 onnx_path 指定策略模型文件")
            raise RuntimeError("onnx_path 参数为空")

        # 初始化 ONNX 推理会话（CPU 推理，实时性足够）
        self.session = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info("已加载策略模型: {}".format(onnx_path))

        # ---- 话题状态 ----
        self.laser = np.ones(NUM_RAYS, dtype=np.float32)  # 重采样后的 40 维激光
        self.goal_xy = None            # 导航目标（来自 /plan 终点）
        self.pose_xy = None            # 机器人当前位置
        self.pose_yaw = 0.0            # 机器人当前朝向
        self.costmap = None            # 最新的局部代价地图（安全盾使用）
        self.last_nav_cmd = None       # 最近一次传统控制器输出（冻结检测依据）
        self.nav_frozen_since = None   # 传统控制器开始冻结的时刻
        self.takeover = False          # RL 接管标志
        self.last_rl_cmd = np.zeros(2, dtype=np.float32)  # 上一周期实际发布的 (v, w)

        # ---- 订阅（/scan 与代价地图使用传感器 QoS）----
        self.create_subscription(LaserScan, "scan", self.scan_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Path, "plan", self.plan_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose",
                                 self.amcl_callback, 10)
        self.create_subscription(OccupancyGrid, "local_costmap/costmap",
                                 self.costmap_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel_nav", self.cmd_vel_nav_callback, 10)

        # ---- 发布 ----
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel_rl", 10)
        self.rl_active_pub = self.create_publisher(Bool, "rl_active", 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, "~/debug_scan", 10)

        # 20Hz 控制循环：冻结检测 -> 推理 -> 安全盾 -> 发布
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("rl_avoidance 节点已启动")

    # ---------- 话题回调 ----------
    def scan_callback(self, msg: LaserScan):
        """任意分辨率激光重采样为 40 维：按角度分 40 桶取每桶最小归一化距离。"""
        laser = np.ones(NUM_RAYS, dtype=np.float32)
        n = len(msg.ranges)
        if n > 0:
            ranges = np.asarray(msg.ranges, dtype=np.float64)
            angles = msg.angle_min + np.arange(n, dtype=np.float64) * msg.angle_increment
            # 有效测距：非 inf/nan 且不小于传感器最小量程；inf/nan 保持默认值 1.0
            valid = np.isfinite(ranges) & (ranges >= msg.range_min)
            # 角度归一化到 [-pi, pi) 后映射到 0~39 号桶（桶中心对齐训练采样）
            wrapped = (angles + math.pi) % (2.0 * math.pi) - math.pi
            buckets = np.clip(((wrapped + math.pi) /
                               (2.0 * math.pi / NUM_RAYS)).astype(np.int64),
                              0, NUM_RAYS - 1)
            # 按最大量程归一化并截断到 1.0，np.minimum.at 逐桶取最小值
            norm = np.minimum(ranges / LASER_RANGE, 1.0).astype(np.float32)
            np.minimum.at(laser, buckets[valid], norm[valid])
        self.laser = laser
        # 调试话题：发布重采样后的 40 维激光，便于与训练观测比对
        dbg = Float32MultiArray()
        dbg.data = laser.tolist()
        self.debug_pub.publish(dbg)

    def plan_callback(self, msg: Path):
        """取全局路径终点作为导航目标。"""
        if msg.poses:
            last = msg.poses[-1].pose.position
            self.goal_xy = np.array([last.x, last.y], dtype=np.float64)

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """更新机器人当前位姿。"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.pose_xy = np.array([pos.x, pos.y], dtype=np.float64)
        self.pose_yaw = yaw_from_quaternion(ori.x, ori.y, ori.z, ori.w)

    def costmap_callback(self, msg: OccupancyGrid):
        """缓存最新的局部代价地图。"""
        self.costmap = msg

    def cmd_vel_nav_callback(self, msg: Twist):
        """缓存传统控制器的速度输出。"""
        self.last_nav_cmd = msg

    # ---------- 主控制循环 ----------
    def control_loop(self):
        now = self.get_clock().now()

        # 当前目标距离（目标与位姿均有效时才有意义）
        goal_dist = None
        if self.goal_xy is not None and self.pose_xy is not None:
            goal_dist = float(np.linalg.norm(self.goal_xy - self.pose_xy))

        # 冻结检测：|v|<0.03 且 |w|<0.03 持续 freeze_timeout 秒，且目标尚远
        nav_frozen = False
        if self.last_nav_cmd is not None and goal_dist is not None:
            v_abs = abs(self.last_nav_cmd.linear.x)
            w_abs = abs(self.last_nav_cmd.angular.z)
            if v_abs < FREEZE_VEL_EPS and w_abs < FREEZE_VEL_EPS \
                    and goal_dist > self.goal_min_dist:
                nav_frozen = True

        if nav_frozen:
            if self.nav_frozen_since is None:
                self.nav_frozen_since = now
            frozen_dur = (now - self.nav_frozen_since).nanoseconds * 1e-9
            if not self.takeover and frozen_dur > self.freeze_timeout:
                self.takeover = True
                self.get_logger().warn(
                    "传统规划器冻结 {:.1f}s 且目标距离 {:.2f}m，RL 接管避障".format(
                        frozen_dur, goal_dist))
        else:
            self.nav_frozen_since = None
            # 传统控制器恢复正常输出（非零速度）或目标已近 -> 交还控制权
            if self.takeover:
                self.get_logger().info("传统控制器恢复正常输出，RL 退出接管")
            self.takeover = False

        # 目标已到达附近 -> 退出接管
        if self.takeover and goal_dist is not None \
                and goal_dist <= self.goal_min_dist:
            self.get_logger().info(
                "目标距离 {:.2f}m 已小于 goal_min_dist，RL 退出接管".format(goal_dist))
            self.takeover = False

        # 推理 + 安全盾 + 发布：危险动作或未接管时一律零速并回退传统控制器
        cmd = self.compute_rl_command() if self.takeover else None
        if cmd is not None and self.check_safety(cmd[0], cmd[1]):
            self.publish_cmd(cmd[0], cmd[1])
            self.publish_active(True)
            self.last_rl_cmd = np.array(cmd, dtype=np.float32)
        else:
            self.publish_cmd(0.0, 0.0)
            self.publish_active(False)
            self.last_rl_cmd = np.zeros(2, dtype=np.float32)

    # ---------- 推理与安全盾 ----------
    def compute_rl_command(self):
        """构造 46 维观测并推理候选速度 (v, w)；位姿或目标缺失时返回 None。"""
        if self.pose_xy is None or self.goal_xy is None:
            return None
        goal_vec = self.goal_xy - self.pose_xy
        goal_dist = float(np.linalg.norm(goal_vec))
        bearing = math.atan2(goal_vec[1], goal_vec[0]) - self.pose_yaw
        bearing = (bearing + math.pi) % (2.0 * math.pi) - math.pi
        aux = np.array([math.sin(bearing), math.cos(bearing), goal_dist / 10.0,
                        float(self.last_rl_cmd[0]) / 0.5,
                        float(self.last_rl_cmd[1]) / 1.2, bearing],
                       dtype=np.float32)
        obs = np.concatenate([self.laser, aux]).astype(np.float32).reshape(1, OBS_DIM)
        action = self.session.run(None, {self.input_name: obs})[0][0]
        v = float(np.clip(action[0], V_MIN, V_MAX))
        w = float(np.clip(action[1], W_MIN, W_MAX))
        return v, w

    def check_safety(self, v: float, w: float) -> bool:
        """安全盾：候选速度沿圆弧轨迹前向投影，逐点检查栅格代价。"""
        if self.costmap is None or self.pose_xy is None:
            # 缺少代价地图或位姿时保守处理：不执行 RL 动作
            return False
        grid = self.costmap
        width, height = grid.info.width, grid.info.height
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        data = grid.data
        x, y, yaw = float(self.pose_xy[0]), float(self.pose_xy[1]), self.pose_yaw
        n_steps = max(1, int(round(self.shield_horizon / self.shield_dt)))
        for k in range(1, n_steps + 1):
            t = k * self.shield_dt
            if abs(w) < 1e-3:
                # 角速度接近零：直线运动
                xk = x + v * t * math.cos(yaw)
                yk = y + v * t * math.sin(yaw)
            else:
                # 差速圆弧运动：先求瞬时圆心，再由旋转角得到投影点
                radius = v / w
                cx = x - radius * math.sin(yaw)
                cy = y + radius * math.cos(yaw)
                yaw_k = yaw + w * t
                xk = cx + radius * math.sin(yaw_k)
                yk = cy - radius * math.cos(yaw_k)
            # 世界坐标转栅格坐标：OccupancyGrid 的 data 行优先且 y 轴向上
            i = int((xk - ox) / res)
            j = int((yk - oy) / res)
            # 投影点在代价地图之外：按不安全处理（保守策略）
            if i < 0 or i >= width or j < 0 or j >= height:
                return False
            if data[j * width + i] > self.occupancy_threshold:
                return False
        return True

    # ---------- 发布辅助 ----------
    def publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def publish_active(self, active: bool):
        msg = Bool()
        msg.data = active
        self.rl_active_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RLAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
