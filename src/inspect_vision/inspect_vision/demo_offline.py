# -*- coding: utf-8 -*-
"""离线演示：不依赖 ROS，直接跑合成图检测（Windows / WSL 均可）。

用法：
  python -m inspect_vision.demo_offline
  或安装后：ros2 run inspect_vision demo_offline
"""

from __future__ import annotations

import argparse
import json
import os
from datetime import datetime

import cv2

from inspect_vision.detector import InspectDetector, make_synthetic_scene


def main():
    parser = argparse.ArgumentParser(description='离线视觉巡检演示')
    parser.add_argument('--out-dir', default='./inspect_results_demo')
    parser.add_argument('--yolo-model', default='')
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    detector = InspectDetector(args.yolo_model)
    tasks = [
        ('F1_P1', 'gauge'),
        ('F1_P2', 'indicator'),
        ('F3_P2', 'extinguisher'),
    ]

    summary = []
    for idx, (name, task) in enumerate(tasks):
        image = make_synthetic_scene(task, seed=10 + idx)
        outcome = detector.detect(image, task)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        img_path = os.path.join(args.out_dir, '%s_%s.jpg' % (name, stamp))
        cv2.imwrite(img_path, outcome.annotated)
        item = {
            'point_name': name,
            'task': task,
            'status': outcome.status,
            'message': outcome.message,
            'confidence': outcome.confidence,
            'image_path': img_path,
        }
        summary.append(item)
        print('%s [%s] %s (%.2f) -> %s' % (
            name, outcome.status, outcome.message, outcome.confidence, img_path))

    json_path = os.path.join(args.out_dir, 'summary.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print('汇总已写入: %s' % json_path)


if __name__ == '__main__':
    main()
