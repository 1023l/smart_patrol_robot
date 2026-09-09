"""跨楼层巡逻任务节点：导航到巡检点后触发视觉巡检。

流程说明：
    读取每层的巡检点配置，逐点调用 Nav2 NavigateToPose；
    到点成功后调用 /inspect_point 做视觉检测并落盘；
    一层完成后调用 /switch_floor 切到下一层（1→2→3→4→5 循环）。
"""

import math
import os
import time

import rclpy
import yaml
from inspect_interfaces.srv import InspectPoint, SwitchMap
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.goal_status import GoalStatus
from rclpy.node import Node


class PatrolMissionNode(Node):
    """跨楼层巡逻 + 到点视觉巡检。"""

    def __init__(self):
        super().__init__('patrol_mission')

        self.declare_parameter('patrol_points_config', '')
        self.declare_parameter('rounds', 2)
        self.declare_parameter('single_point_timeout', 120.0)
        self.declare_parameter('switch_floor_timeout', 30.0)
        self.declare_parameter('enable_vision', True)
        self.declare_parameter('inspect_timeout', 15.0)

        config_path = self.get_parameter(
            'patrol_points_config').get_parameter_value().string_value
        self._rounds = int(self.get_parameter('rounds').get_parameter_value().integer_value)
        self._point_timeout = float(self.get_parameter(
            'single_point_timeout').get_parameter_value().double_value)
        self._switch_timeout = float(self.get_parameter(
            'switch_floor_timeout').get_parameter_value().double_value)
        self._enable_vision = bool(
            self.get_parameter('enable_vision').get_parameter_value().bool_value)
        self._inspect_timeout = float(self.get_parameter(
            'inspect_timeout').get_parameter_value().double_value)

        self._patrol_points = self._load_patrol_points(config_path)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._switch_client = self.create_client(SwitchMap, '/switch_floor')
        self._inspect_client = self.create_client(InspectPoint, '/inspect_point')

    def _load_patrol_points(self, config_path):
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
                    'task': str(point.get('task', 'gauge')),
                })
            if point_list:
                points_map[floor_id] = point_list

        if not points_map:
            raise RuntimeError('巡检点配置为空：%s' % config_path)
        return points_map

    def run_patrol(self):
        total_count = 0
        nav_success = 0
        nav_failed = 0
        inspect_normal = 0
        inspect_abnormal = 0
        inspect_unknown = 0
        start_time = time.monotonic()

        floor_sequence = sorted(self._patrol_points.keys())
        self.get_logger().info(
            '巡逻任务开始：楼层顺序 %s，共 %d 轮，视觉巡检=%s' % (
                floor_sequence, self._rounds, self._enable_vision))

        current_floor = None
        for round_index in range(1, self._rounds + 1):
            self.get_logger().info(
                '========== 第 %d / %d 轮巡逻开始 ==========' % (round_index, self._rounds))
            for floor in floor_sequence:
                points = self._patrol_points[floor]

                if current_floor != floor:
                    ok, message = self._switch_floor(floor)
                    if not ok:
                        self.get_logger().error(
                            '切换到楼层 %d 失败：%s，跳过该层 %d 个巡检点' % (
                                floor, message, len(points)))
                        total_count += len(points)
                        nav_failed += len(points)
                        continue
                    current_floor = floor
                    self.get_logger().info('已切换到楼层 %d' % floor)

                for point in points:
                    total_count += 1
                    self.get_logger().info(
                        '开始导航：楼层 %d 巡检点 %s (%.2f, %.2f, yaw=%.2f, task=%s)' % (
                            floor, point['name'], point['x'], point['y'],
                            point['yaw'], point['task']))
                    if not self._navigate_to_point(point):
                        nav_failed += 1
                        self.get_logger().warn('巡检点 %s 导航失败' % point['name'])
                        continue

                    nav_success += 1
                    self.get_logger().info('巡检点 %s 导航成功' % point['name'])

                    if not self._enable_vision:
                        continue
                    status, message = self._inspect_point(floor, point)
                    if status == 'NORMAL':
                        inspect_normal += 1
                    elif status == 'ABNORMAL':
                        inspect_abnormal += 1
                    else:
                        inspect_unknown += 1
                    self.get_logger().info(
                        '视觉巡检 %s => %s (%s)' % (point['name'], status, message))

            self.get_logger().info(
                '========== 第 %d 轮巡逻结束 ==========' % round_index)

        elapsed = time.monotonic() - start_time
        self.get_logger().info(
            '巡逻任务全部结束：总点=%d 导航成功=%d 导航失败=%d '
            '检测正常=%d 异常=%d 未知=%d 耗时=%.1fs' % (
                total_count, nav_success, nav_failed,
                inspect_normal, inspect_abnormal, inspect_unknown, elapsed))

    def _switch_floor(self, target_floor):
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

    def _inspect_point(self, floor, point):
        if not self._inspect_client.wait_for_service(timeout_sec=5.0):
            return 'UNKNOWN', '/inspect_point 不可用（可先启动 inspect_vision）'

        request = InspectPoint.Request()
        request.point_name = point['name']
        request.floor = int(floor)
        request.task = point.get('task', 'gauge')
        future = self._inspect_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._inspect_timeout)
        response = future.result()
        if response is None:
            return 'UNKNOWN', '调用 /inspect_point 超时'
        if not response.success:
            return 'UNKNOWN', response.message
        return response.status, response.message

    def _navigate_to_point(self, point):
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose 动作服务器不可用：navigate_to_pose')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(point['x'])
        goal.pose.pose.position.y = float(point['y'])
        goal.pose.pose.position.z = 0.0

        half_yaw = float(point['yaw']) * 0.5
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(half_yaw)
        goal.pose.pose.orientation.w = math.cos(half_yaw)

        start_time = time.monotonic()

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

        result_future = goal_handle.get_result_async()
        remaining = self._point_timeout - (time.monotonic() - start_time)
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=max(remaining, 0.0))

        result = result_future.result()
        if result is None:
            self.get_logger().error(
                '导航执行超时（%.0f 秒），已取消该目标' % self._point_timeout)
            goal_handle.cancel_goal_async()
            return False

        return result.status == GoalStatus.STATUS_SUCCEEDED


def main(args=None):
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
