# -*- coding: utf-8 -*-
"""
速度多路选择节点：在传统控制器与 RL 避障之间做最终速度切换。

切换规则：
- /rl_active 为 True 时，把 /cmd_vel_rl 转发到 /cmd_vel_raw；
- /rl_active 为 False 时，把 /cmd_vel_nav 转发到 /cmd_vel_raw；
- 超过 0.5 秒未收到 /rl_active 消息时，安全默认回退传统控制器。
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

# rl_active 消息超时时间（秒），超时后安全回退传统控制器
RL_ACTIVE_TIMEOUT = 0.5


class VelocityMuxNode(Node):
    """速度多路选择器：rl_active 有效且为 True 时转发 RL 速度，否则转发传统速度。"""

    def __init__(self):
        super().__init__("velocity_mux")

        self.rl_active = False            # 当前是否处于 RL 接管状态
        self.last_rl_active_time = None    # 最近一次收到 rl_active 的时刻
        self.last_nav_cmd = None           # 缓存的最近一次传统控制器速度

        self.create_subscription(Twist, "cmd_vel_nav", self.nav_callback, 10)
        self.create_subscription(Twist, "cmd_vel_rl", self.rl_callback, 10)
        self.create_subscription(Bool, "rl_active", self.rl_active_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_raw", 10)

        # 10Hz 超时监测
        self.timer = self.create_timer(0.1, self.timeout_check)
        self.get_logger().info("velocity_mux 已启动：默认转发 /cmd_vel_nav")

    def nav_callback(self, msg: Twist):
        """传统控制器速度：仅在 RL 未接管时转发。"""
        self.last_nav_cmd = msg
        if not self.rl_active:
            self.cmd_pub.publish(msg)

    def rl_callback(self, msg: Twist):
        """RL 速度：仅在 RL 接管期间转发。"""
        if self.rl_active:
            self.cmd_pub.publish(msg)

    def rl_active_callback(self, msg: Bool):
        """RL 接管状态更新；退出接管时立即无缝切回传统控制器。"""
        self.last_rl_active_time = self.get_clock().now()
        if msg.data and not self.rl_active:
            self.get_logger().info("RL 接管：转发 /cmd_vel_rl")
        elif not msg.data and self.rl_active:
            self.get_logger().info("RL 退出接管：转发 /cmd_vel_nav")
            # 立即补发一次传统控制器最近速度，避免速度空档
            if self.last_nav_cmd is not None:
                self.cmd_pub.publish(self.last_nav_cmd)
        self.rl_active = msg.data

    def timeout_check(self):
        """超过 0.5 秒未收到 rl_active 消息则回退传统控制器（安全默认）。"""
        if not self.rl_active:
            return
        if self.last_rl_active_time is None:
            self.rl_active = False
            return
        elapsed = (self.get_clock().now() -
                   self.last_rl_active_time).nanoseconds * 1e-9
        if elapsed > RL_ACTIVE_TIMEOUT:
            self.get_logger().warn(
                "rl_active 消息超时 {:.2f}s，安全回退传统控制器".format(elapsed))
            self.rl_active = False
            if self.last_nav_cmd is not None:
                self.cmd_pub.publish(self.last_nav_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityMuxNode()
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
