# -*- coding: utf-8 -*-
"""巡检视觉检测器：优先可选 YOLO，默认 OpenCV 规则，保证无模型也能跑通。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class DetectionOutcome:
    status: str          # NORMAL / ABNORMAL / UNKNOWN
    message: str
    confidence: float
    annotated: np.ndarray


def make_synthetic_scene(task: str, seed: int = 0) -> np.ndarray:
    """生成合成巡检画面，便于无相机时联调。"""
    rng = np.random.default_rng(seed)
    img = np.full((480, 640, 3), 40, dtype=np.uint8)
    cv2.rectangle(img, (40, 40), (600, 440), (70, 70, 70), -1)

    task = (task or 'gauge').lower()
    if task == 'gauge':
        center = (320, 240)
        cv2.circle(img, center, 90, (200, 200, 200), 3)
        # 指针角度：多数正常，偶发越界视为异常
        angle = float(rng.uniform(-50, 50))
        if rng.random() < 0.25:
            angle = float(rng.choice([-80, 80]))
        rad = np.deg2rad(angle - 90)
        tip = (int(center[0] + 70 * np.cos(rad)), int(center[1] + 70 * np.sin(rad)))
        cv2.line(img, center, tip, (0, 0, 255), 5)
        cv2.circle(img, tip, 4, (0, 0, 255), -1)
        cv2.putText(img, 'GAUGE', (40, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2)
    elif task == 'indicator':
        # 绿=正常，红=异常
        abnormal = rng.random() < 0.3
        color = (0, 0, 220) if abnormal else (0, 200, 0)
        cv2.circle(img, (320, 240), 55, color, -1)
        cv2.putText(img, 'INDICATOR', (40, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2)
    else:  # extinguisher
        present = rng.random() >= 0.2
        if present:
            cv2.rectangle(img, (260, 120), (380, 380), (0, 40, 200), -1)
            cv2.rectangle(img, (300, 90), (340, 120), (180, 180, 180), -1)
        cv2.putText(img, 'EXTINGUISHER', (40, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2)
    return img


class InspectDetector:
    """OpenCV 规则检测 + 可选 Ultralytics YOLO。"""

    def __init__(self, yolo_model_path: str = '', yolo_conf: float = 0.35):
        self.yolo_conf = float(yolo_conf)
        self._yolo = None
        if yolo_model_path:
            try:
                from ultralytics import YOLO  # type: ignore
                self._yolo = YOLO(yolo_model_path)
            except Exception:
                self._yolo = None

    def detect(self, image_bgr: np.ndarray, task: str) -> DetectionOutcome:
        task = (task or 'gauge').lower()
        if self._yolo is not None:
            outcome = self._detect_yolo(image_bgr, task)
            if outcome is not None:
                return outcome
        if task == 'gauge':
            return self._detect_gauge(image_bgr)
        if task == 'indicator':
            return self._detect_indicator(image_bgr)
        return self._detect_extinguisher(image_bgr)

    def _detect_yolo(self, image_bgr: np.ndarray, task: str) -> Optional[DetectionOutcome]:
        try:
            results = self._yolo.predict(image_bgr, conf=self.yolo_conf, verbose=False)
        except Exception:
            return None
        if not results:
            return None
        annotated = results[0].plot()
        names = results[0].names or {}
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return DetectionOutcome('UNKNOWN', 'YOLO 未检出目标', 0.2, annotated)

        labels = []
        confs = []
        for box in boxes:
            cls_id = int(box.cls.item())
            conf = float(box.conf.item())
            labels.append(str(names.get(cls_id, cls_id)))
            confs.append(conf)

        joined = ','.join(labels).lower()
        mean_conf = float(np.mean(confs)) if confs else 0.0
        if 'abnormal' in joined or 'alarm' in joined or 'fire' in joined:
            return DetectionOutcome('ABNORMAL', 'YOLO 检出异常: %s' % joined, mean_conf, annotated)
        return DetectionOutcome('NORMAL', 'YOLO 检出: %s' % joined, mean_conf, annotated)

    def _detect_gauge(self, image_bgr: np.ndarray) -> DetectionOutcome:
        annotated = image_bgr.copy()
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=80,
            param1=100, param2=40, minRadius=40, maxRadius=140)
        if circles is None:
            return DetectionOutcome('UNKNOWN', '未找到仪表圆盘', 0.2, annotated)

        circles = np.uint16(np.around(circles))
        x, y, r = [int(v) for v in circles[0, 0]]
        cv2.circle(annotated, (x, y), r, (0, 255, 0), 2)

        # 优先用红色指针像素估计方向（合成图与多数红针仪表适用）
        tip = self._estimate_red_needle_tip(image_bgr, x, y, r)
        if tip is None:
            # 回退：圆盘 ROI 内最长直线
            mask = np.zeros_like(gray)
            cv2.circle(mask, (x, y), int(r * 0.9), 255, -1)
            edges = cv2.Canny(gray, 50, 150)
            edges = cv2.bitwise_and(edges, edges, mask=mask)
            lines = cv2.HoughLinesP(
                edges, 1, np.pi / 180, threshold=30,
                minLineLength=max(12, int(r * 0.35)), maxLineGap=10)
            if lines is None:
                return DetectionOutcome('UNKNOWN', '找到表盘但未识别指针', 0.35, annotated)
            best = max(
                lines[:, 0],
                key=lambda ln: (ln[0] - ln[2]) ** 2 + (ln[1] - ln[3]) ** 2)
            x1, y1, x2, y2 = [int(v) for v in best]
            d1 = (x1 - x) ** 2 + (y1 - y) ** 2
            d2 = (x2 - x) ** 2 + (y2 - y) ** 2
            tip = (x1, y1) if d1 > d2 else (x2, y2)

        cv2.line(annotated, (x, y), tip, (0, 0, 255), 2)
        angle = np.degrees(np.arctan2(tip[1] - y, tip[0] - x)) + 90.0
        angle = (angle + 180.0) % 360.0 - 180.0
        if abs(angle) > 60.0:
            return DetectionOutcome(
                'ABNORMAL', '仪表指针越界 angle=%.1fdeg' % angle, 0.75, annotated)
        return DetectionOutcome(
            'NORMAL', '仪表读数正常 angle=%.1fdeg' % angle, 0.8, annotated)

    @staticmethod
    def _estimate_red_needle_tip(
            image_bgr: np.ndarray, cx: int, cy: int, radius: int
    ) -> Optional[Tuple[int, int]]:
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        red1 = cv2.inRange(hsv, (0, 80, 80), (12, 255, 255))
        red2 = cv2.inRange(hsv, (168, 80, 80), (180, 255, 255))
        red = cv2.bitwise_or(red1, red2)
        mask = np.zeros(red.shape, dtype=np.uint8)
        cv2.circle(mask, (cx, cy), int(radius * 0.95), 255, -1)
        cv2.circle(mask, (cx, cy), max(2, int(radius * 0.08)), 0, -1)
        red = cv2.bitwise_and(red, red, mask=mask)
        ys, xs = np.where(red > 0)
        if len(xs) < 8:
            return None
        # 取离圆心最远的红色像素作为针尖
        d2 = (xs - cx) ** 2 + (ys - cy) ** 2
        idx = int(np.argmax(d2))
        return int(xs[idx]), int(ys[idx])

    def _detect_indicator(self, image_bgr: np.ndarray) -> DetectionOutcome:
        annotated = image_bgr.copy()
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        # 红灯（两段色相）与绿灯
        red1 = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255))
        red2 = cv2.inRange(hsv, (170, 80, 80), (180, 255, 255))
        red = cv2.bitwise_or(red1, red2)
        green = cv2.inRange(hsv, (40, 60, 60), (90, 255, 255))
        red_ratio = float(np.count_nonzero(red)) / red.size
        green_ratio = float(np.count_nonzero(green)) / green.size

        if red_ratio > 0.01 and red_ratio >= green_ratio:
            ys, xs = np.where(red > 0)
            cv2.rectangle(annotated, (int(xs.min()), int(ys.min())),
                          (int(xs.max()), int(ys.max())), (0, 0, 255), 2)
            return DetectionOutcome(
                'ABNORMAL', '指示灯为红色（告警） red=%.3f' % red_ratio, 0.85, annotated)
        if green_ratio > 0.01:
            ys, xs = np.where(green > 0)
            cv2.rectangle(annotated, (int(xs.min()), int(ys.min())),
                          (int(xs.max()), int(ys.max())), (0, 255, 0), 2)
            return DetectionOutcome(
                'NORMAL', '指示灯为绿色 green=%.3f' % green_ratio, 0.85, annotated)
        return DetectionOutcome('UNKNOWN', '未识别到指示灯颜色', 0.25, annotated)

    def _detect_extinguisher(self, image_bgr: np.ndarray) -> DetectionOutcome:
        annotated = image_bgr.copy()
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        red1 = cv2.inRange(hsv, (0, 70, 70), (12, 255, 255))
        red2 = cv2.inRange(hsv, (168, 70, 70), (180, 255, 255))
        red = cv2.bitwise_or(red1, red2)
        red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w = image_bgr.shape[:2]
        min_area = 0.01 * h * w
        candidates = [c for c in contours if cv2.contourArea(c) >= min_area]
        if not candidates:
            return DetectionOutcome('ABNORMAL', '灭火器区域未检测到红色目标', 0.7, annotated)
        best = max(candidates, key=cv2.contourArea)
        x, y, bw, bh = cv2.boundingRect(best)
        cv2.rectangle(annotated, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        return DetectionOutcome(
            'NORMAL', '检测到灭火器候选框 area=%d' % int(cv2.contourArea(best)), 0.75, annotated)
