"""跨楼层巡逻任务节点：按楼层顺序执行巡检点导航，并联动楼层切换。

流程说明：
    读取每层的巡检点配置，逐点调用 Nav2 的 NavigateToPose 动作完成导航；
    一层全部巡检点完成后调用 /switch_floor 切换到下一层，
    楼层按 1→2→3→4→5→1 循环，全部轮次结束后打印任务统计。
"""

import math
import os
import time

import rclpy
import yaml
from inspect_interfaces.srv import SwitchMap
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.goal_status import GoalStatus
from rclpy.node import Node


class PatrolMissionNode(Node):
    """跨楼层巡逻任务节点。"""

    def __init__(self):
        super().__init__('patrol_mission')

        # 巡检点配置 yaml 路径
        self.declare_parameter('patrol_points_config', '')
        # 巡逻轮数（完整遍历一遍全部楼层记为一轮）
        self.declare_parameter('rounds', 2)
        # 单个巡检点导航超时时间（秒）
        self.declare_parameter('single_point_timeout', 120.0)
        # 楼层切换服务调用超时时间（秒）
        self.declare_parameter('switch_floor_timeout', 30.0)

        config_path = self.get_parameter(
            'patrol_points_config').get_parameter_value().string_value
        self._rounds = int(self.get_parameter('rounds').get_parameter_value().integer_value)
        self._point_timeout = float(self.get_parameter(
            'single_point_timeout').get_parameter_value().double_value)
        self._switch_timeout = float(self.get_parameter(
            'switch_floor_timeout').get_parameter_value().double_value)

        # 读取巡检点配置：{楼层号: [{name, x, y, yaw}, ...]}
        self._patrol_points = self._load_patrol_points(config_path)

        # NavigateToPose 动作客户端（Nav2 导航）
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # 楼层切换服务客户端
        self._switch_client = self.create_client(SwitchMap, '/switch_floor')

    def _load_patrol_points(self, config_path):
        """读取巡检点配置 yaml，返回 {楼层号: [巡检点字典列表]} 形式的数据。"""
        if not config_path:
            raise RuntimeError('参数 patrol_points_config 未配置，无法加载巡检点')
        if not os.path.isfile(config_path):
            raise RuntimeError('巡检点配置文件不存在：%s' % config_path)

        with open(config_path, 'r', encoding='utf-8') as config_file:
            data = yaml.safe_load(config_file)

        if not isinstance(data, dict) or 'patrol_points' not in data:
            raise RuntimeError('巡检点配置格式错误：缺少 patrol_points 字段：%s' % config_path)

        points_map = {}
        for key, points in (data['patrol_points'] or {}).items():
            floor_id = int(key)
            point_list = []
            for point in points or []:
                point_list.append({
                    'name': str(point.get('name', 'F%d_P%d' % (
                        floor_id, len(point_list) + 1))),
                    'x': float(point['x']),
                    'y': float(point['y']),
                    'yaw': float(point.get('yaw', 0.0)),
                })
            if point_list:
                points_map[floor_id] = point_list

        if not points_map:
            raise RuntimeError('巡检点配置为空：%s' % config_path)
        return points_map

    def run_patrol(self):
        """执行跨楼层巡逻主流程，全部结束后打印统计信息。"""
        total_count = 0
        success_count = 0
        failed_count = 0
        start_time = time.monotonic()

        # 楼层顺序（1→2→3→4→5，跨轮时自然衔接 5→1 的回环切换）
        floor_sequence = sorted(self._patrol_points.keys())
        self.get_logger().info(
            '巡逻任务开始：楼层顺序 %s，共 %d 轮' % (floor_sequence, self._rounds))

        current_floor = None
        for round_index in range(1, self._rounds + 1):
            self.get_logger().info(
                '========== 第 %d / %d 轮巡逻开始 ==========' % (round_index, self._rounds))
            for floor in floor_sequence:
                points = self._patrol_points[floor]

                # 目标楼层与当前楼层不一致时，先切换楼层
                if current_floor != floor:
                    ok, message = self._switch_floor(floor)
                    if not ok:
                        # 切层失败：该层全部巡检点记为失败，继续下一层
                        self.get_logger().error(
                            '切换到楼层 %d 失败：%s，跳过该层 %d 个巡检点' % (
                                floor, message, len(points)))
                        total_count += len(points)
                        failed_count += len(points)
                        continue
                    current_floor = floor
                    self.get_logger().info('已切换到楼层 %d' % floor)

                # 逐点导航
                for point in points:
                    total_count += 1
                    self.get_logger().info(
                        '开始导航：楼层 %d 巡检点 %s (%.2f, %.2f, yaw=%.2f)' % (
                            floor, point['name'], point['x'], point['y'], point['yaw']))
                    if self._navigate_to_point(point):
                        success_count += 1
                        self.get_logger().info('巡检点 %s 导航成功' % point['name'])
                    else:
                        failed_count += 1
                        self.get_logger().warn('巡检点 %s 导航失败' % point['name'])
            self.get_logger().info(
                '========== 第 %d 轮巡逻结束 ==========' % round_index)

        elapsed = time.monotonic() - start_time
        self.get_logger().info(
            '巡逻任务全部结束，统计：总任务数=%d，成功=%d，失败=%d，总耗时=%.1f 秒' % (
                total_count, success_count, failed_count, elapsed))

    def _switch_floor(self, target_floor):
        """调用 /switch_floor 服务切换楼层，返回 (是否成功, 说明信息) 元组。"""
        if not self._switch_client.wait_for_service(timeout_sec=10.0):
            return False, '/switch_floor 服务不可用：请确认 floor_manager 节点已启动'

        request = SwitchMap.Request()
        request.target_floor = target_floor
        future = self._switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._switch_timeout)

        response = future.result()
        if response is None:
            return False, '调用 /switch_floor 超时（%.0f 秒）' % self._switch_timeout
        if not response.success:
            return False, response.message
        return True, response.message

    def _navigate_to_point(self, point):
        """向 Nav2 发送 NavigateToPose 目标并等待结果，返回该点是否导航成功。"""
        # 等待导航动作服务器就绪
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose 动作服务器不可用：navigate_to_pose')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(point['x'])
        goal.pose.pose.position.y = float(point['y'])
        goal.pose.pose.position.z = 0.0

        # yaw（绕 z 轴旋转）转四元数
        half_yaw = float(point['yaw']) * 0.5
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(half_yaw)
        goal.pose.pose.orientation.w = math.cos(half_yaw)

        # 单点总超时控制：从发送目标开始计时
        start_time = time.monotonic()

        # 发送目标并等待动作服务器接受
        send_future = self._nav_client.send_goal_async(goal)
        remaining = self._point_timeout - (time.monotonic() - start_time)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=max(remaining, 0.0))

        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error('发送导航目标超时或失败')
            return False
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            return False

        # 等待导航执行结果
        result_future = goal_handle.get_result_async()
        remaining = self._point_timeout - (time.monotonic() - start_time)
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=max(remaining, 0.0))

        result = result_future.result()
        if result is None:
            # 执行超时：取消该目标，避免影响后续任务
            self.get_logger().error(
                '导航执行超时（%.0f 秒），已取消该目标' % self._point_timeout)
            goal_handle.cancel_goal_async()
            return False

        return result.status == GoalStatus.STATUS_SUCCEEDED


def main(args=None):
    """节点入口：构造巡逻节点并执行巡逻流程。"""
    rclpy.init(args=args)
    node = PatrolMissionNode()
    try:
        node.run_patrol()
    except KeyboardInterrupt:
        node.get_logger().info('巡逻任务被手动中断')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
