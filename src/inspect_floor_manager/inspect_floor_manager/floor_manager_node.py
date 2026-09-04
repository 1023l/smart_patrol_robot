"""楼层管理节点：负责分层子地图切换与电梯口 AMCL 重定位。

架构说明：
    五层工业厂房采用分层独立子地图方案（每层一张栅格地图）。
    机器人搭乘电梯到达目标楼层后，通过本节点加载该层地图，
    并在电梯出入口发布初始位姿触发 AMCL 重定位，完成跨楼层巡检。
"""

import math
import os
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from inspect_interfaces.srv import SwitchMap
from nav2_msgs.srv import LoadMap
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class FloorManagerNode(Node):
    """楼层管理节点：提供 /switch_floor 与 /relocalize 服务。"""

    def __init__(self):
        super().__init__('floor_manager')

        # 地图目录参数（一般由 launch 注入 inspect_navigation 包 share 目录下的 maps/）
        self.declare_parameter('maps_dir', '')
        # 楼层配置 yaml 路径参数
        self.declare_parameter('floors_config', '')

        self._maps_dir = self.get_parameter('maps_dir').get_parameter_value().string_value
        floors_config = self.get_parameter('floors_config').get_parameter_value().string_value

        # 读取楼层配置：{楼层号: {map: 地图yaml文件名, elevator: {x, y, yaw}}}
        self._floors = self._load_floors_config(floors_config)

        # 当前楼层（初始未知，首次切换成功后开始记录）
        self._current_floor = None

        # 服务回调内部需要同步等待 /map_server/load_map 的响应，
        # 因此使用可重入回调组配合多线程执行器，避免死锁。
        self._cb_group = ReentrantCallbackGroup()

        # /switch_floor 服务：切换楼层（加载新地图并在电梯口重定位）
        self._switch_srv = self.create_service(
            SwitchMap, '/switch_floor', self._handle_switch_floor,
            callback_group=self._cb_group)

        # /relocalize 服务：在当前楼层电梯点重新发布初始位姿
        self._relocalize_srv = self.create_service(
            Trigger, '/relocalize', self._handle_relocalize,
            callback_group=self._cb_group)

        # /map_server/load_map 服务客户端
        # 注意：该服务只在 map_server 处于 active 生命周期状态时可用
        self._load_map_client = self.create_client(
            LoadMap, '/map_server/load_map', callback_group=self._cb_group)

        # /initialpose 发布者：发布初始位姿触发 AMCL 重定位
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.get_logger().info(
            '楼层管理节点已启动：共加载 %d 层配置，maps_dir=%s' % (
                len(self._floors), self._maps_dir))

    def _load_floors_config(self, config_path):
        """读取楼层配置 yaml，返回 {楼层号: 配置字典} 形式的数据。"""
        if not config_path:
            raise RuntimeError('参数 floors_config 未配置，无法加载楼层配置')
        if not os.path.isfile(config_path):
            raise RuntimeError('楼层配置文件不存在：%s' % config_path)

        with open(config_path, 'r', encoding='utf-8') as config_file:
            data = yaml.safe_load(config_file)

        if not isinstance(data, dict) or 'floors' not in data:
            raise RuntimeError('楼层配置格式错误：缺少 floors 字段：%s' % config_path)

        floors = {}
        for key, cfg in (data['floors'] or {}).items():
            if not isinstance(cfg, dict) or 'map' not in cfg or 'elevator' not in cfg:
                raise RuntimeError('楼层 %s 配置不完整：需要 map 与 elevator 字段' % key)
            elevator = cfg['elevator']
            floors[int(key)] = {
                'map': str(cfg['map']),
                'elevator': {
                    'x': float(elevator['x']),
                    'y': float(elevator['y']),
                    'yaw': float(elevator.get('yaw', 0.0)),
                },
            }

        if not floors:
            raise RuntimeError('楼层配置为空：%s' % config_path)
        return floors

    def _handle_switch_floor(self, request, response):
        """处理 /switch_floor 请求：加载目标楼层地图并在电梯口重定位。"""
        target_floor = int(request.target_floor)

        # a) 校验目标楼层存在，且与当前楼层不同
        if target_floor not in self._floors:
            response.success = False
            response.message = '目标楼层 %d 不在楼层配置中，可用楼层：%s' % (
                target_floor, sorted(self._floors.keys()))
            return response
        if self._current_floor is not None and target_floor == self._current_floor:
            response.success = False
            response.message = '目标楼层 %d 与当前楼层相同，无需切换' % target_floor
            return response

        floor_cfg = self._floors[target_floor]

        # 校验地图目录与目标地图文件
        if not self._maps_dir:
            response.success = False
            response.message = '参数 maps_dir 未配置，无法定位地图文件'
            return response
        map_path = os.path.join(self._maps_dir, floor_cfg['map'])
        if not os.path.isfile(map_path):
            response.success = False
            response.message = '目标楼层地图文件不存在：%s' % map_path
            return response

        # b) 调用 /map_server/load_map 加载目标楼层地图
        # 注意：load_map 服务只在 map_server 处于 active 生命周期状态时可用，
        # 若 map_server 未启动或处于非 active 状态，此处会失败并把原因写入 message。
        if not self._load_map_client.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = ('/map_server/load_map 服务不可用：'
                                '请确认 map_server 已启动且处于 active 生命周期状态')
            return response

        self.get_logger().info('开始切换到楼层 %d，加载地图：%s' % (target_floor, map_path))
        load_request = LoadMap.Request()
        load_request.map_url = map_path
        load_future = self._load_map_client.call_async(load_request)
        if not self._wait_future(load_future, timeout_sec=10.0):
            response.success = False
            response.message = '调用 /map_server/load_map 超时（10 秒）'
            return response

        load_response = load_future.result()
        if load_response is None or load_response.result != LoadMap.Response.RESULT_SUCCESS:
            result_code = load_response.result if load_response is not None else None
            response.success = False
            response.message = '加载地图失败（load_map 返回 result=%s）：%s' % (
                result_code, map_path)
            return response

        # c) 等待静态地图层与 AMCL 接收到新地图
        time.sleep(1.0)

        # d) 在目标楼层电梯口发布初始位姿，触发 AMCL 重定位
        elevator = floor_cfg['elevator']
        self._publish_initialpose(elevator['x'], elevator['y'], elevator['yaw'])
        self.get_logger().info(
            '已在楼层 %d 电梯口 (%.2f, %.2f, yaw=%.2f) 发布初始位姿' % (
                target_floor, elevator['x'], elevator['y'], elevator['yaw']))

        # e) 等待 AMCL 重定位收敛后返回成功
        time.sleep(2.0)

        self._current_floor = target_floor
        response.success = True
        response.message = '已成功切换到楼层 %d' % target_floor
        self.get_logger().info(response.message)
        return response

    def _handle_relocalize(self, request, response):
        """处理 /relocalize 请求：在当前楼层电梯点重新发布初始位姿。"""
        if self._current_floor is None:
            response.success = False
            response.message = '当前楼层未知，请先调用 /switch_floor 完成楼层切换'
            return response

        elevator = self._floors[self._current_floor]['elevator']
        self._publish_initialpose(elevator['x'], elevator['y'], elevator['yaw'])
        # 等待 AMCL 重定位收敛后再返回
        time.sleep(2.0)

        response.success = True
        response.message = '已在楼层 %d 电梯点重新发布初始位姿并等待重定位收敛' % (
            self._current_floor)
        self.get_logger().info(response.message)
        return response

    def _publish_initialpose(self, x, y, yaw):
        """发布 /initialpose 消息（frame_id 为 map），触发 AMCL 重定位。

        协方差矩阵为 6x6（顺序为 x, y, z, roll, pitch, yaw），
        按需求在 x、y、yaw 三个对角项上取 0.05。
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        # yaw（绕 z 轴旋转）转四元数
        half_yaw = float(yaw) * 0.5
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        # 协方差对角项：x、y、yaw 均为 0.05
        msg.pose.covariance[0] = 0.05   # x 方差
        msg.pose.covariance[7] = 0.05   # y 方差
        msg.pose.covariance[35] = 0.05  # yaw 方差

        self._initialpose_pub.publish(msg)

    def _wait_future(self, future, timeout_sec):
        """在服务回调上下文中等待 future 完成。

        服务回调由执行器线程调用，不能再调用 spin_until_future_complete，
        这里借助完成事件阻塞等待，配合多线程执行器处理响应。
        """
        done_event = threading.Event()
        future.add_done_callback(lambda _future: done_event.set())
        return done_event.wait(timeout_sec)


def main(args=None):
    """节点入口：使用多线程执行器运行楼层管理节点。"""
    rclpy.init(args=args)
    node = FloorManagerNode()

    # 多线程执行器：服务回调阻塞等待 load_map 响应时，
    # 其他线程仍可处理服务响应，避免死锁。
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
