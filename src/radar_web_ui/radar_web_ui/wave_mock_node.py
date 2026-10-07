# -*- coding: utf-8 -*-
"""解析波模拟发布节点.

用于在没有真实解析波模块时验证 Web UI 第 2 页(解析波分析)的显示与接口。
发布 /wave_analysis (std_msgs/msg/String, JSON 字符串),字段与 WAVE_SCHEMA 一致。

用法:
    ros2 run radar_web_ui wave_mock_node --period 0.2 --mode fmcw
"""
from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _build_samples(mode: str, seq: int, count: int = 512) -> list:
    samples = []
    if mode == "fmcw":
        # 模拟 FMCW 差频信号: 三个距离目标 -> 三个正弦叠加
        ranges = [(6.0, 0.8), (9.5, 0.45), (14.2, 0.22)]
        for i in range(count):
            t_ms = i * 0.01
            t_s = t_ms / 1000.0
            value = 0.0
            for dist, amp in ranges:
                freq = 40.0 + dist * 18.0  # 差频随距离变化
                value += amp * math.sin(2 * math.pi * freq * t_s)
            value += 0.06 * math.sin(2 * math.pi * 7.0 * t_s)  # 载波泄漏
            noise = math.sin(i * 1.7) * 0.03
            samples.append({
                "t_ms": round(t_ms, 3),
                "amplitude": round(value + noise, 4),
                "phase_deg": round(math.degrees(math.atan2(value, 1.0)), 2),
                "frequency_hz": round(40.0 + 9.5 * 18.0, 2),
                "snr_db": round(18.0 + 2.0 * math.sin(seq * 0.3), 2),
            })
    elif mode == "pulse":
        # 模拟脉冲雷达: 距离门 + 角度扫描
        for i in range(count):
            t_ms = i * 0.02
            dist = 5.0 + ((i * 1.3 + seq * 3.0) % 16.0)
            center = 8.5 + 3.0 * math.sin(seq * 0.25)
            amp = max(0.0, math.exp(-0.5 * ((dist - center) / 1.1) ** 2)) * (0.7 + 0.3 * math.sin(seq))
            samples.append({
                "t_ms": round(t_ms, 3),
                "amplitude": round(amp + 0.04 * math.sin(i * 2.3), 4),
                "distance_m": round(dist, 3),
                "angle_deg": round(math.degrees(math.atan2(center - 7.5, 18.0)), 2),
                "snr_db": round(10.0 + 10.0 * amp, 2),
            })
    else:
        for i in range(count):
            t_ms = i * 0.05
            value = math.sin(2 * math.pi * 3.0 * t_ms / 1000.0) + 0.3 * math.sin(2 * math.pi * 8.0 * t_ms / 1000.0)
            samples.append({"t_ms": round(t_ms, 3), "amplitude": round(value, 4)})
    return samples


class WaveMockNode(Node):
    def __init__(self, mode: str = "fmcw", period: float = 0.2, count: int = 512):
        super().__init__("wave_mock_node")
        self.mode = mode
        self.count = int(count)
        self.seq = 0
        self.publisher = self.create_publisher(String, "wave_analysis", 10)
        self.create_timer(float(period), self.publish_tick)
        self.get_logger().info(
            f"解析波模拟已启动: mode={mode} period={period}s count={count} topic=/wave_analysis")

    def publish_tick(self) -> None:
        now = time.time()
        samples = _build_samples(self.mode, self.seq, self.count)
        payload = {
            "source": "wave_mock_node",
            "seq": self.seq,
            "status": "valid",
            "mode": self.mode,
            "meta": {
                "sample_rate_hz": round(self.count / 0.25, 1),
                "center_frequency_hz": 10.0e9,
                "bandwidth_hz": 1.0e9,
                "note": "mock data for radar_web_ui wave page",
            },
            "samples": samples,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.publisher.publish(msg)
        self.seq += 1


def main():
    import argparse

    parser = argparse.ArgumentParser(description="解析波模拟发布节点")
    parser.add_argument("--mode", choices=["fmcw", "pulse", "cw"], default="fmcw")
    parser.add_argument("--period", type=float, default=0.2)
    parser.add_argument("--count", type=int, default=512)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = WaveMockNode(mode=args.mode, period=args.period, count=args.count)
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
