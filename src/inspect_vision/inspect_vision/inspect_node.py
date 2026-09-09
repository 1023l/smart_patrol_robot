# -*- coding: utf-8 -*-
"""到点视觉巡检节点：提供 /inspect_point 服务，发布 /inspect/result。"""

from __future__ import annotations

import json
import os
import threading
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from inspect_interfaces.msg import InspectResult
from inspect_interfaces.srv import InspectPoint
from inspect_vision.detector import InspectDetector, make_synthetic_scene


class InspectNode(Node):
    """抓取最新图像（或合成图）→ 检测 → 落盘 → 返回结果。"""

    def __init__(self):
        super().__init__('inspect_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('image_timeout_sec', 2.0)
        self.declare_parameter('allow_synthetic', True)
        self.declare_parameter('results_dir', '/tmp/inspect_results')
        self.declare_parameter('yolo_model_path', '')
        self.declare_parameter('yolo_conf', 0.35)
        self.declare_parameter('publish_debug_image', True)

        self._image_topic = self.get_parameter('image_topic').value
        self._image_timeout = float(self.get_parameter('image_timeout_sec').value)
        self._allow_synthetic = bool(self.get_parameter('allow_synthetic').value)
        self._results_dir = str(self.get_parameter('results_dir').value)
        yolo_path = str(self.get_parameter('yolo_model_path').value)
        yolo_conf = float(self.get_parameter('yolo_conf').value)
        self._publish_debug = bool(self.get_parameter('publish_debug_image').value)

        os.makedirs(self._results_dir, exist_ok=True)

        self._bridge = CvBridge()
        self._detector = InspectDetector(yolo_path, yolo_conf)
        self._latest_image = None
        self._latest_stamp = None
        self._image_lock = threading.Lock()
        self._cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Image, self._image_topic, self._on_image, 10,
            callback_group=self._cb_group)

        self._result_pub = self.create_publisher(InspectResult, '/inspect/result', 10)
        self._result_json_pub = self.create_publisher(String, '/inspect/result_json', 10)
        self._debug_pub = self.create_publisher(Image, '/inspect/debug_image', 10)

        self._srv = self.create_service(
            InspectPoint, '/inspect_point', self._handle_inspect,
            callback_group=self._cb_group)

        self.get_logger().info(
            '视觉巡检节点已启动：image_topic=%s results_dir=%s synthetic=%s' % (
                self._image_topic, self._results_dir, self._allow_synthetic))

    def _on_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn('图像转换失败: %s' % exc)
            return
        with self._image_lock:
            self._latest_image = frame
            self._latest_stamp = self.get_clock().now()

    def _get_image(self, task: str, point_name: str) -> tuple:
        """返回 (bgr图像, 来源说明)。优先真实相机，超时可用合成图。"""
        import time as _time
        deadline = _time.monotonic() + self._image_timeout
        while _time.monotonic() < deadline:
            with self._image_lock:
                if self._latest_image is not None:
                    return self._latest_image.copy(), 'camera:%s' % self._image_topic
            _time.sleep(0.05)

        if not self._allow_synthetic:
            raise RuntimeError('等待图像超时且不允许合成图：%s' % self._image_topic)

        seed = abs(hash(point_name)) % (2 ** 31)
        return make_synthetic_scene(task, seed=seed), 'synthetic'

    def _handle_inspect(self, request, response):
        point_name = request.point_name or 'UNKNOWN'
        task = (request.task or 'gauge').lower()
        floor = int(request.floor)

        try:
            image, source = self._get_image(task, point_name)
            outcome = self._detector.detect(image, task)

            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            base = 'F%d_%s_%s' % (floor, point_name, stamp)
            img_path = os.path.join(self._results_dir, base + '.jpg')
            json_path = os.path.join(self._results_dir, base + '.json')
            cv2.imwrite(img_path, outcome.annotated)

            payload = {
                'point_name': point_name,
                'floor': floor,
                'task': task,
                'status': outcome.status,
                'message': outcome.message,
                'confidence': float(outcome.confidence),
                'image_path': img_path,
                'image_source': source,
                'timestamp': stamp,
            }
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)

            msg = InspectResult()
            msg.point_name = point_name
            msg.floor = floor
            msg.task = task
            msg.status = outcome.status
            msg.message = outcome.message
            msg.image_path = img_path
            msg.confidence = float(outcome.confidence)
            msg.stamp = self.get_clock().now().to_msg()
            self._result_pub.publish(msg)

            js = String()
            js.data = json.dumps(payload, ensure_ascii=False)
            self._result_json_pub.publish(js)

            if self._publish_debug:
                debug = self._bridge.cv2_to_imgmsg(outcome.annotated, encoding='bgr8')
                debug.header.stamp = msg.stamp
                self._debug_pub.publish(debug)

            response.success = True
            response.status = outcome.status
            response.message = '%s | source=%s | saved=%s' % (
                outcome.message, source, img_path)
            response.image_path = img_path
            response.confidence = float(outcome.confidence)
            self.get_logger().info(
                '巡检完成 %s floor=%d status=%s' % (point_name, floor, outcome.status))
        except Exception as exc:
            response.success = False
            response.status = 'UNKNOWN'
            response.message = '巡检失败: %s' % exc
            response.image_path = ''
            response.confidence = 0.0
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = InspectNode()
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
