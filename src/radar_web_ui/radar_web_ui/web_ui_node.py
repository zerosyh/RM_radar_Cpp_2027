# -*- coding: utf-8 -*-
"""RM_radar_Cpp_2027 调试 Web UI 节点.

一个纯 Python(标准库)ROS2 节点,负责:
  1. 订阅现有雷达话题,聚合为 JSON 状态;
  2. 通过 HTTP + SSE 把状态推给浏览器;
  3. 提供解析波(Wave Analysis)接入接口,供未来解析波模块对接;
  4. 提供调试参数读写接口。

启动:
    ros2 run radar_web_ui web_ui_node --port 8766 --config configs/main_config.yaml
浏览器打开 http://<本机IP>:8766
"""
from __future__ import annotations

import argparse
import json
import mimetypes
import os
import signal
import sys
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - 开发环境未 source install 时兜底
    get_package_share_directory = None

import yaml

from detect_result.msg import DetectResult, EkfDiagnostics, EkfDiagnosticsArray, Location, Locations, Robots
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header, String

try:
    import cv2
    import numpy as np
    _CV_AVAILABLE = True
except Exception:  # pragma: no cover - cv2/numpy 缺失时图片接口降级
    cv2 = None
    np = None
    _CV_AVAILABLE = False


# --------------------------------------------------------------------------- #
# 常量 / 默认值
# --------------------------------------------------------------------------- #
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8766
MAX_BODY_BYTES = 8 * 1024 * 1024
MAX_WAVE_SAMPLES = 8192
MAX_WAVE_HISTORY = 40
MAX_LOG_EVENTS = 120
SSE_KEEPALIVE_SEC = 15.0
MJPEG_INTERVAL_SEC = 0.033

DEFAULT_DEBUG_PARAMS: Dict[str, Any] = {
    "map_image_enabled": True,
    "map_show_grid": True,
    "map_show_labels": True,
    "trail_enabled": True,
    "trail_max_points": 120,
    "show_last_known": True,
    "robot_marker_radius_px": 16,
    "air_marker_radius_px": 13,
    "stale_timeout_s": 3.0,
    "detection_keep_count": 60,
    "wave_max_samples": MAX_WAVE_SAMPLES,
    "wave_history_keep": MAX_WAVE_HISTORY,
}

WAVE_SCHEMA: Dict[str, Any] = {
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "RM_radar_Cpp_2027 解析波接口数据",
    "description": "解析波模块通过 ROS2 话题 /wave_analysis (std_msgs/String, JSON 字符串) "
                   "或 HTTP POST /api/wave/ingest 推送解析结果。",
    "type": "object",
    "required": ["samples"],
    "properties": {
        "source": {"type": "string", "description": "解析波来源节点/设备名, 例如 wave_parser"},
        "seq": {"type": "integer", "description": "帧序号"},
        "status": {"type": "string", "enum": ["idle", "running", "valid", "invalid", "warn"],
                   "description": "解析状态"},
        "mode": {"type": "string", "description": "工作模式, 例如 CW/FMCW/脉冲"},
        "meta": {"type": "object", "description": "任意扩展字段(频率/带宽/通道等)"},
        "samples": {
            "type": "array",
            "minItems": 1,
            "items": {
                "type": "object",
                "properties": {
                    "t_ms": {"type": "number", "description": "相对时间 ms"},
                    "amplitude": {"type": "number"},
                    "phase_deg": {"type": "number"},
                    "frequency_hz": {"type": "number"},
                    "snr_db": {"type": "number"},
                    "distance_m": {"type": "number"},
                    "angle_deg": {"type": "number"},
                    "valid": {"type": "boolean"}
                }
            }
        }
    }
}

_JSON_TYPES = (str, int, float, bool, type(None))


def _jsonable(obj: Any, _depth: int = 0) -> Any:
    """把 numpy 标量等非标准类型安全地转成 JSON 可序列化对象."""
    if _depth > 8:
        return str(obj)
    if obj is None or isinstance(obj, _JSON_TYPES):
        return obj
    if isinstance(obj, dict):
        return {str(k): _jsonable(v, _depth + 1) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v, _depth + 1) for v in obj]
    if hasattr(obj, "tolist"):
        try:
            return _jsonable(obj.tolist(), _depth + 1)
        except Exception:
            pass
    if hasattr(obj, "item"):
        try:
            return _jsonable(obj.item(), _depth + 1)
        except Exception:
            pass
    return str(obj)


def _to_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _ros_stamp_sec(stamp: Any) -> float:
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except Exception:
        return 0.0


def _now() -> float:
    return time.time()


def _pid_memory_rss_kb(pid: Optional[int] = None) -> int:
    pid = pid or os.getpid()
    try:
        text = Path(f"/proc/{pid}/status").read_text(encoding="utf-8")
        for line in text.splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1])
    except Exception:
        pass
    return 0


def _read_yaml(path: Path) -> Dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        return data if isinstance(data, dict) else {}
    except Exception as exc:
        return {"__error__": f"读取失败: {exc}"}


def _clamp(value: Any, low: float, high: float, default: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return max(low, min(high, number))


# --------------------------------------------------------------------------- #
# 解析波数据归一化
# --------------------------------------------------------------------------- #
class WaveStore:
    """保存未来解析波模块推送的数据. 输入格式宽容,输出格式统一."""

    _SAMPLE_ALIASES = {
        "t": "t_ms", "time": "t_ms", "time_ms": "t_ms", "timestamp_ms": "t_ms",
        "amp": "amplitude", "value": "amplitude", "voltage": "amplitude",
        "phase": "phase_deg", "snr": "snr_db", "freq": "frequency_hz",
        "freq_hz": "frequency_hz", "range": "distance_m", "range_m": "distance_m",
        "dist": "distance_m", "distance": "distance_m",
        "angle": "angle_deg", "azimuth": "angle_deg", "azimuth_deg": "angle_deg",
    }

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.received_count = 0
        self.received_bytes = 0
        self.history: deque = deque(maxlen=MAX_WAVE_HISTORY)
        self.latest: Optional[Dict[str, Any]] = None
        self.last_error: Optional[str] = None

    def _normalize_sample(self, raw: Any, index: int) -> Optional[Dict[str, Any]]:
        if isinstance(raw, (int, float)):
            return {"index": index, "t_ms": float(index), "amplitude": float(raw)}
        if not isinstance(raw, dict):
            return None
        sample: Dict[str, Any] = {"index": index}
        for key, value in raw.items():
            if key in ("index", "valid") and isinstance(value, bool):
                sample["valid"] = value
                continue
            canonical = self._SAMPLE_ALIASES.get(key, key)
            if isinstance(value, bool):
                sample[canonical] = value
            elif isinstance(value, (int, float)):
                sample[canonical] = float(value)
            elif value is None:
                continue
            else:
                try:
                    sample[canonical] = float(value)
                except (TypeError, ValueError):
                    sample[canonical] = str(value)
        if "valid" not in sample:
            sample["valid"] = True
        return sample

    def ingest(self, payload: Any, source: str, byte_count: int = 0) -> Dict[str, Any]:
        """接收一个 JSON 载荷,归一化后存入 latest + history."""
        now = _now()
        with self.lock:
            self.received_count += 1
            self.received_bytes += int(byte_count or 0)
            self.last_error = None

            if isinstance(payload, list):
                raw_samples = payload
                meta: Dict[str, Any] = {}
                payload_status = "valid"
                seq = None
                payload_source = source
                mode = None
            elif isinstance(payload, dict):
                raw_samples = payload.get("samples", payload.get("data", []))
                meta = payload.get("meta") if isinstance(payload.get("meta"), dict) else {}
                payload_status = str(payload.get("status", "valid")).lower()
                seq = payload.get("seq")
                payload_source = str(payload.get("source", source))
                mode = str(payload.get("mode", "")) or None
            else:
                self.last_error = "载荷必须是 JSON object 或 array"
                return {"ok": False, "error": self.last_error}

            if not isinstance(raw_samples, list):
                self.last_error = "缺少 samples 数组"
                return {"ok": False, "error": self.last_error}

            max_samples = DEFAULT_DEBUG_PARAMS["wave_max_samples"]
            samples: List[Dict[str, Any]] = []
            for i, raw in enumerate(raw_samples[:max_samples]):
                sample = self._normalize_sample(raw, i)
                if sample is not None:
                    samples.append(sample)
            if not samples:
                self.last_error = "samples 为空或全部无法解析"
                return {"ok": False, "error": self.last_error}

            try:
                seq_int = None if seq is None else int(seq)
            except (TypeError, ValueError):
                seq_int = None
            latest: Dict[str, Any] = {
                "source": payload_source,
                "status": payload_status,
                "mode": mode,
                "seq": seq_int,
                "meta": meta,
                "received_at": now,
                "sample_count": len(samples),
                "dropped_count": max(0, len(raw_samples) - len(samples)),
                "samples": samples,
            }
            self.latest = latest
            self.history.append({
                "source": payload_source,
                "status": payload_status,
                "mode": mode,
                "seq": latest["seq"],
                "received_at": now,
                "sample_count": len(samples),
                "meta": meta,
            })
            return {
                "ok": True,
                "stored": True,
                "sample_count": len(samples),
                "dropped_count": latest["dropped_count"],
            }

    def clear(self) -> Dict[str, Any]:
        with self.lock:
            self.latest = None
            self.history.clear()
            self.received_count = 0
            self.received_bytes = 0
            self.last_error = None
            return {"ok": True}

    def snapshot(self, max_samples: Optional[int] = None) -> Dict[str, Any]:
        with self.lock:
            if not self.latest:
                return {
                    "available": False,
                    "message": "等待解析波模块接入。通过 HTTP POST /api/wave/ingest 或 ROS2 话题 /wave_analysis 推送数据。",
                    "history": list(self.history),
                    "received_count": self.received_count,
                    "last_error": self.last_error,
                }
            latest = dict(self.latest)
            samples = latest.get("samples", [])
            if max_samples and len(samples) > int(max_samples):
                latest["samples"] = samples[: int(max_samples)]
            return {
                "available": True,
                "latest": latest,
                "history": list(self.history),
                "received_count": self.received_count,
                "received_bytes": self.received_bytes,
                "last_error": self.last_error,
            }


# --------------------------------------------------------------------------- #
# ROS2 节点
# --------------------------------------------------------------------------- #
class RadarWebUiNode(Node):
    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT,
                 config_path: str = "configs/main_config.yaml") -> None:
        super().__init__("radar_web_ui")

        self.declare_parameter("host", host)
        self.declare_parameter("port", port)
        self.declare_parameter("config_path", config_path)
        host = str(self.get_parameter("host").value or DEFAULT_HOST)
        port = int(self.get_parameter("port").value or DEFAULT_PORT)
        config_path = str(self.get_parameter("config_path").value or config_path)

        self.host = host
        self.port = port
        self.config_path = self._resolve_config_path(config_path)
        self.project_root = self._resolve_project_root(self.config_path)

        self.start_time = _now()
        self.generation = 0
        self._lock = threading.RLock()
        self._stream_condition = threading.Condition(self._lock)
        self._shutdown = threading.Event()

        # 聚合状态
        self.topic_stats: Dict[str, Dict[str, Any]] = {}
        self.robot_states: Dict[int, Dict[str, Any]] = {}
        self.sentry_entries: List[Dict[str, Any]] = []
        self.ai_nav_entries: List[Dict[str, Any]] = []
        self.detections: List[Dict[str, Any]] = []
        self.ekf_slots: List[Dict[str, Any]] = []
        self.image_frames: Dict[str, Dict[str, Any]] = {}
        self.image_sequence: Dict[str, int] = {"map_view": 0, "detect_view": 0}
        self.sse_clients: set = set()
        self.log_events: deque = deque(maxlen=MAX_LOG_EVENTS)
        self.ros_graph: Dict[str, Any] = {"updated_at": 0.0, "topics": []}
        self.config_summary: Dict[str, Any] = {}
        self.wave_store = WaveStore()
        self.debug_params = dict(DEFAULT_DEBUG_PARAMS)
        self._load_debug_params()

        self.web_root = self._resolve_web_root()
        self.config_summary = self._summarize_config()
        self._log("info", f"Web UI 初始化完成: http://{host}:{port} config={self.config_path}")

        self._setup_subscriptions()
        self.create_timer(1.0, self._metrics_tick)
        self.create_timer(5.0, self._ros_graph_tick)

    # ----------------------------- 路径/配置 ----------------------------- #
    @staticmethod
    def _resolve_config_path(config_path: str) -> Path:
        candidates = [Path(config_path)]
        cwd = Path.cwd()
        candidates.append(cwd / "configs" / "main_config.yaml")
        home_root = Path.home() / "rm_lidar_2027" / "RM_radar_Cpp_2027"
        candidates.append(home_root / "configs" / "main_config.yaml")
        candidates.append(home_root / "configs" / "main_config_lab.yaml")
        for candidate in candidates:
            if candidate.is_file():
                return candidate.resolve()
        return Path(config_path).resolve()

    @staticmethod
    def _resolve_project_root(config_path: Path) -> Path:
        root = config_path.parent.parent
        if (root / "configs" / "main_config.yaml").is_file():
            return root.resolve()
        cwd = Path.cwd()
        if (cwd / "configs" / "main_config.yaml").is_file():
            return cwd.resolve()
        home_root = Path.home() / "rm_lidar_2027" / "RM_radar_Cpp_2027"
        if (home_root / "configs" / "main_config.yaml").is_file():
            return home_root.resolve()
        return cwd.resolve()

    @staticmethod
    def _resolve_web_root() -> Path:
        if get_package_share_directory is not None:
            try:
                installed = Path(get_package_share_directory("radar_web_ui")) / "web"
                if (installed / "index.html").is_file():
                    return installed
            except Exception:
                pass
        source = Path(__file__).resolve().parents[1] / "web"
        return source if (source / "index.html").is_file() else Path(__file__).resolve().parent

    def _summarize_config(self) -> Dict[str, Any]:
        cfg = _read_yaml(self.config_path)
        if "__error__" in cfg:
            return {"file": str(self.config_path), "error": cfg["__error__"], "scene": {}, "global": {},
                    "camera": {}, "lidar": {}, "car": {}, "ai": {}, "map": {}}
        scene_name = str(((cfg.get("global") or {}).get("scene")) or "competition")
        scenes = cfg.get("scenes") or {}
        scene = scenes.get(scene_name) or {}
        lidar = cfg.get("lidar") or {}
        return {
            "file": str(self.config_path),
            "project_root": str(self.project_root),
            "global": cfg.get("global") or {},
            "camera": cfg.get("camera") or {},
            "lidar": lidar,
            "car": cfg.get("car") or {},
            "ai": cfg.get("ai") or {},
            "scene": {
                "name": scene_name,
                "field_width": _to_float(scene.get("field_width"), 28.0),
                "field_height": _to_float(scene.get("field_height"), 15.0),
                "std_map": scene.get("std_map"),
                "pcd_file": scene.get("pcd_file"),
            },
            "map_path": str(self._resolve_relative_path(scene.get("std_map", "")) or ""),
        }

    def _resolve_relative_path(self, rel: str) -> Optional[Path]:
        if not rel:
            return None
        path = Path(rel)
        if path.is_absolute():
            return path if path.is_file() else None
        for base in (self.project_root, Path.cwd()):
            candidate = base / rel
            if candidate.is_file():
                return candidate.resolve()
        return None

    def _load_debug_params(self) -> None:
        candidates = [
            Path.home() / ".config" / "rm_radar_web_ui" / "debug_params.json",
            self.project_root / "web_ui_debug_params.json",
        ]
        for path in candidates:
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                if isinstance(data, dict):
                    for key, value in data.items():
                        if key in DEFAULT_DEBUG_PARAMS:
                            self.debug_params[key] = value
                    self._log("info", f"已加载调试参数: {path}")
                    return
            except FileNotFoundError:
                continue
            except Exception:
                continue

    def _save_debug_params(self) -> None:
        try:
            path = Path.home() / ".config" / "rm_radar_web_ui" / "debug_params.json"
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(self.debug_params, ensure_ascii=False, indent=2),
                            encoding="utf-8")
        except Exception:
            pass

    # ----------------------------- 日志/事件 ----------------------------- #
    def _log(self, level: str, message: str, detail: Optional[Dict[str, Any]] = None) -> None:
        event = {
            "timestamp": _now(),
            "level": level,
            "message": message,
            "detail": _jsonable(detail) if detail else None,
        }
        with self._lock:
            self.log_events.appendleft(event)
            self._notify_locked()
        getattr(self.get_logger(), {"debug": "debug", "info": "info", "warn": "warn",
                                    "error": "error"}.get(level, "info"))(message)

    def _notify_locked(self) -> None:
        """调用方必须持有 self._lock."""
        self.generation += 1
        self._stream_condition.notify_all()

    def _publish_state(self, reason: str = "") -> None:
        with self._lock:
            self._notify_locked()

    # ----------------------------- ROS 订阅 ----------------------------- #
    def _topic_touch(self, name: str, msg_type: str, wall: float,
                     header_sec: float = 0.0, extra: Optional[Dict[str, Any]] = None) -> None:
        with self._lock:
            entry = self.topic_stats.setdefault(name, {
                "name": name,
                "type": msg_type,
                "count": 0,
                "first_wall": wall,
                "last_wall": wall,
                "last_header_sec": header_sec,
                "timestamps": deque(maxlen=600),
                "extra": {},
            })
            entry["count"] += 1
            entry["last_wall"] = wall
            entry["timestamps"].append(wall)
            if header_sec:
                entry["last_header_sec"] = header_sec
            if extra:
                entry["extra"].update(extra)
            self._notify_locked()

    @staticmethod
    def _qos(depth: int) -> QoSProfile:
        qos = QoSProfile(depth=int(depth))
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        return qos

    def _setup_subscriptions(self) -> None:
        qos_loc = self._qos(10)
        qos_sensor = self._qos(5)
        qos_img = self._qos(2)

        self.sub_location = self.create_subscription(
            Locations, "location", lambda msg: self._location_cb(msg, "location"), qos_loc)
        self.sub_ekf = self.create_subscription(
            Locations, "ekf_location_filtered", lambda msg: self._location_cb(msg, "ekf_location_filtered"), qos_loc)
        self.sub_sentry = self.create_subscription(
            Locations, "sentry_targets", lambda msg: self._locations_list_cb(msg, "sentry_targets"), qos_loc)
        self.sub_ai_nav = self.create_subscription(
            Locations, "ai_nav", lambda msg: self._locations_list_cb(msg, "ai_nav"), qos_loc)
        self.sub_detect = self.create_subscription(
            Robots, "detect_result", self._detect_cb, qos_sensor)
        self.sub_map_view = self.create_subscription(
            Image, "map_view", lambda msg: self._image_cb(msg, "map_view"), qos_img)
        self.sub_detect_view = self.create_subscription(
            Image, "detect_view", lambda msg: self._image_cb(msg, "detect_view"), qos_img)
        self.sub_target_pcd = self.create_subscription(
            PointCloud2, "target_pointcloud", lambda msg: self._pcd_cb(msg, "target_pointcloud"), qos_loc)
        self.sub_other_pcd = self.create_subscription(
            PointCloud2, "livox/lidar_other", lambda msg: self._pcd_cb(msg, "livox/lidar_other"), qos_loc)
        self.sub_wave = self.create_subscription(
            String, "wave_analysis", self._wave_ros_cb, qos_loc)
        # 兼容未来解析波节点使用 best_effort QoS 发布的情况
        qos_best = QoSProfile(depth=10)
        qos_best.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_best.durability = DurabilityPolicy.VOLATILE
        self.sub_wave_best = self.create_subscription(
            String, "wave_analysis", self._wave_ros_cb, qos_best)
        self.sub_ekf_diag = self.create_subscription(
            EkfDiagnosticsArray, "ekf_diagnostics", self._ekf_diag_cb, qos_loc)

        self.get_logger().info(
            "已订阅: location / ekf_location_filtered / sentry_targets / ai_nav / detect_result "
            "/ map_view / detect_view / target_pointcloud / livox/lidar_other / wave_analysis / ekf_diagnostics")

    def _location_cb(self, msg: Locations, source: str) -> None:
        now = _now()
        locs = self._location_list(msg)
        with self._lock:
            for item in locs:
                robot_id = int(item["id"])
                entry = self.robot_states.setdefault(robot_id, {
                    "id": robot_id,
                    "first_seen": now,
                    "last_seen": now,
                    "message_count": 0,
                    "current": None,
                    "trail": deque(maxlen=int(self.debug_params.get("trail_max_points", 120))),
                })
                entry["message_count"] += 1
                entry["last_seen"] = now
                item["source"] = source
                item["age"] = 0.0
                entry["current"] = item
                if robot_id < 0:
                    item["last_known"] = True
                entry["trail"].append({"t": now, "x": item["x"], "y": item["y"], "source": source})
                # 同步 id 正负号: 负 id 表示 last_known
                entry["last_known"] = robot_id < 0
            self._topic_touch(source, "detect_result/msg/Locations", now)

    @staticmethod
    def _location_list(msg: Locations) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for loc in msg.locs:
            out.append({
                "id": int(loc.id),
                "label": str(loc.label),
                "x": _to_float(loc.x),
                "y": _to_float(loc.y),
                "z": _to_float(loc.z),
            })
        return out

    def _locations_list_cb(self, msg: Locations, source: str) -> None:
        now = _now()
        items = []
        for item in self._location_list(msg):
            item["source"] = source
            item["age"] = 0.0
            items.append(item)
        with self._lock:
            if source == "sentry_targets":
                self.sentry_entries = items
            elif source == "ai_nav":
                self.ai_nav_entries = items
            self._topic_touch(source, "detect_result/msg/Locations", now)
            self._notify_locked()

    def _detect_cb(self, msg: Robots) -> None:
        now = _now()
        items: List[Dict[str, Any]] = []
        for det in msg.detect_results:
            items.append({
                "track_id": int(det.track_id),
                "label": str(det.label),
                "confidence": _to_float(det.confidence),
                "xyxy_box": [int(v) for v in det.xyxy_box],
                "xywh_box": [_to_float(v) for v in det.xywh_box],
                "field_x": _to_float(det.field_x),
                "field_y": _to_float(det.field_y),
                "field_z": _to_float(det.field_z),
            })
        with self._lock:
            keep = int(self.debug_params.get("detection_keep_count", 60))
            self.detections = items[:keep]
            self.detections_timestamp = now
            self._topic_touch("detect_result", "detect_result/msg/Robots", now,
                              extra={"detections": len(items)})
            self._notify_locked()

    def _ekf_diag_cb(self, msg: EkfDiagnosticsArray) -> None:
        now = _now()
        slots: List[Dict[str, Any]] = []
        for slot in msg.slots:
            slots.append({
                "slot_id": int(slot.slot_id),
                "robot_id": int(slot.robot_id),
                "detection_rate_hz": _to_float(slot.detection_rate_hz),
                "time_since_last_det_ms": _to_float(slot.time_since_last_det_ms),
                "jitter": _to_float(slot.jitter),
                "innovation_norm": _to_float(slot.innovation_norm),
                "raw_vs_filtered_dist": _to_float(slot.raw_vs_filtered_dist),
                "covariance_trace": _to_float(slot.covariance_trace),
            })
        with self._lock:
            self.ekf_slots = slots
            self._topic_touch("ekf_diagnostics", "detect_result/msg/EkfDiagnosticsArray",
                              now, header_sec=_ros_stamp_sec(msg.stamp))
            self._notify_locked()

    def _pcd_cb(self, msg: PointCloud2, source: str) -> None:
        point_count = int(msg.width) * int(msg.height) if int(msg.width) > 0 else int(msg.row_step)
        with self._lock:
            self._topic_touch(source, "sensor_msgs/msg/PointCloud2", _now(),
                              header_sec=_ros_stamp_sec(msg.header.stamp),
                              extra={"points": point_count, "fields": len(msg.fields)})
            self._notify_locked()

    def _image_cb(self, msg: Image, source: str) -> None:
        if not _CV_AVAILABLE:
            with self._lock:
                self._topic_touch(source, "sensor_msgs/msg/Image", _now(),
                                  header_sec=_ros_stamp_sec(msg.header.stamp),
                                  extra={"error": "cv2/numpy 不可用, 无法编码"})
                self._notify_locked()
            return
        try:
            jpeg = self._encode_image(msg)
        except Exception as exc:
            with self._lock:
                self._topic_touch(source, "sensor_msgs/msg/Image", _now(),
                                  header_sec=_ros_stamp_sec(msg.header.stamp),
                                  extra={"error": str(exc)})
                self._notify_locked()
            return
        now = _now()
        with self._lock:
            frame = {
                "timestamp": now,
                "header_sec": _ros_stamp_sec(msg.header.stamp),
                "width": int(msg.width),
                "height": int(msg.height),
                "encoding": str(msg.encoding),
                "jpeg_bytes": bytes(jpeg),
                "jpeg_size": len(jpeg),
                "frame_id": str(msg.header.frame_id),
            }
            self.image_frames[source] = frame
            self.image_sequence[source] = int(self.image_sequence.get(source, 0)) + 1
            self._topic_touch(source, "sensor_msgs/msg/Image", now,
                              header_sec=frame["header_sec"],
                              extra={"width": frame["width"], "height": frame["height"],
                                     "jpeg_size": frame["jpeg_size"]})
            self._notify_locked()

    @staticmethod
    def _encode_image(msg: Image) -> bytes:
        """ROS Image -> JPEG bytes(供 MJPEG/快照使用)."""
        if np is None or cv2 is None:
            raise RuntimeError("numpy/cv2 不可用")
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        height, width = int(msg.height), int(msg.width)
        encoding = str(msg.encoding)
        try:
            if encoding in ("bgr8", "8UC3"):
                mat = raw.reshape(height, width, 3)
            elif encoding in ("rgb8",):
                mat = raw.reshape(height, width, 3)[:, :, ::-1]
            elif encoding in ("mono8", "8UC1"):
                mat = raw.reshape(height, width)
            elif encoding in ("bgra8",):
                mat = cv2.cvtColor(raw.reshape(height, width, 4), cv2.COLOR_BGRA2BGR)
            elif encoding in ("rgba8",):
                mat = cv2.cvtColor(raw.reshape(height, width, 4), cv2.COLOR_RGBA2BGR)
            elif encoding in ("mono16", "16UC1"):
                mat16 = raw.reshape(height, width).astype(np.float32)
                mat16 = np.clip(mat16 / 256.0, 0, 255).astype(np.uint8)
                mat = mat16
            else:
                # 未知编码: 若数据长度匹配 bgr8 则按 bgr8 处理
                mat = raw.reshape(height, width, 3)
        except Exception:
            mat = raw.reshape(max(1, raw.size), 1, 1).repeat(3, axis=2)
        ok, buf = cv2.imencode(".jpg", mat, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
        if not ok:
            raise RuntimeError("JPEG 编码失败")
        return buf.tobytes()

    def _wave_ros_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self._log("warn", f"/wave_analysis 收到非 JSON 数据: {exc}")
            return
        result = self.wave_store.ingest(payload, "ros2:/wave_analysis",
                                        byte_count=len(msg.data))
        with self._lock:
            self._topic_touch("wave_analysis", "std_msgs/msg/String", _now())
            if result.get("ok"):
                self._log("info", f"解析波已接入: {result.get('sample_count')} 个采样点")
            else:
                self._log("warn", f"解析波数据被拒绝: {result.get('error')}")
            self._notify_locked()

    # ----------------------------- 定时任务 ----------------------------- #
    def _metrics_tick(self) -> None:
        now = _now()
        with self._lock:
            # 丢弃过长的 trail
            trail_max = int(self.debug_params.get("trail_max_points", 120))
            for entry in self.robot_states.values():
                if entry["trail"].maxlen != trail_max:
                    entry["trail"] = deque(entry["trail"], maxlen=trail_max)
            self._notify_locked()

    def _ros_graph_tick(self) -> None:
        try:
            names_and_types = self.get_topic_names_and_types()
            topics = [{"name": n, "types": list(t)} for n, t in sorted(names_and_types)]
            with self._lock:
                self.ros_graph = {"updated_at": _now(), "topics": topics}
                self._notify_locked()
        except Exception as exc:
            self.get_logger().debug(f"ROS graph 查询失败: {exc}")

    # ----------------------------- 状态快照 ----------------------------- #
    def _topic_snapshot(self) -> List[Dict[str, Any]]:
        now = _now()
        out = []
        for name, entry in self.topic_stats.items():
            stamps = list(entry["timestamps"])
            rate = 0.0
            if len(stamps) >= 2:
                span = stamps[-1] - stamps[0]
                if span > 0.01:
                    rate = (len(stamps) - 1) / span
            out.append({
                "name": name,
                "type": entry["type"],
                "count": entry["count"],
                "last_wall": entry["last_wall"],
                "age": max(0.0, now - entry["last_wall"]),
                "last_header_sec": entry.get("last_header_sec", 0.0),
                "rate_hz": round(rate, 2),
                "extra": _jsonable(entry.get("extra", {})),
            })
        out.sort(key=lambda item: item["name"])
        return out

    def _robot_snapshot(self) -> List[Dict[str, Any]]:
        now = _now()
        stale_timeout = float(self.debug_params.get("stale_timeout_s", 3.0))
        out = []
        for robot_id, entry in self.robot_states.items():
            current = dict(entry["current"]) if entry["current"] else None
            if current is not None:
                current["age"] = max(0.0, now - entry["last_seen"])
                current["stale"] = current["age"] > stale_timeout
            trail = []
            for point in entry["trail"]:
                trail.append({"t": round(point["t"] - self.start_time, 3),
                              "x": round(point["x"], 3),
                              "y": round(point["y"], 3),
                              "source": point.get("source", "")})
            out.append({
                "id": robot_id,
                "first_seen_age": round(now - entry["first_seen"], 3),
                "last_seen_age": round(now - entry["last_seen"], 3),
                "message_count": entry["message_count"],
                "last_known": bool(entry.get("last_known", False)),
                "current": current,
                "trail": trail,
            })
        out.sort(key=lambda item: (bool(item.get("last_known")), -item["last_seen_age"]))
        return out

    def snapshot(self) -> Dict[str, Any]:
        now = _now()
        with self._lock:
            images = {}
            for name, frame in self.image_frames.items():
                images[name] = {
                    "available": True,
                    "updated_at": frame["timestamp"],
                    "header_sec": frame["header_sec"],
                    "width": frame["width"],
                    "height": frame["height"],
                    "encoding": frame["encoding"],
                    "jpeg_size": frame["jpeg_size"],
                    "frame_id": frame["frame_id"],
                    "sequence": self.image_sequence.get(name, 0),
                }
            for name in ("map_view", "detect_view"):
                images.setdefault(name, {"available": False, "sequence": 0})

            payload = {
                "ok": True,
                "now": now,
                "generation": self.generation,
                "node": {
                    "name": self.get_name(),
                    "namespace": self.get_namespace(),
                    "start_time": self.start_time,
                    "uptime": round(now - self.start_time, 1),
                    "pid": os.getpid(),
                    "memory_rss_kb": _pid_memory_rss_kb(),
                    "host": self.host,
                    "port": self.port,
                    "sse_clients": len(self.sse_clients),
                    "threads": threading.active_count(),
                },
                "config": self.config_summary,
                "debug": _jsonable(dict(self.debug_params)),
                "topics": self._topic_snapshot(),
                "robots": self._robot_snapshot(),
                "sentry": _jsonable(list(self.sentry_entries)),
                "ai_nav": _jsonable(list(self.ai_nav_entries)),
                "detections": _jsonable(list(self.detections)),
                "detections_timestamp": getattr(self, "detections_timestamp", 0.0),
                "ekf_slots": _jsonable(list(self.ekf_slots)),
                "images": images,
                # SSE/HTTP 快照最多携带 2048 点(表格只显示 16 行,图表 2048 点足够),
                # 完整数据仍保留在 WaveStore 内,避免多客户端时推送超大 JSON。
                "wave": _jsonable(self.wave_store.snapshot(
                    max_samples=min(int(self.debug_params.get("wave_max_samples", MAX_WAVE_SAMPLES)), 2048))),
                "logs": list(self.log_events),
                "ros_graph": {
                    "updated_at": self.ros_graph.get("updated_at", 0.0),
                    "topic_count": len(self.ros_graph.get("topics", [])),
                    "topics": self.ros_graph.get("topics", [])[:200],
                },
            }
            return payload

    def wait_snapshot(self, after_generation: int, timeout: float = SSE_KEEPALIVE_SEC) -> Dict[str, Any]:
        with self._stream_condition:
            self._stream_condition.wait_for(
                lambda: self.generation > after_generation or self._shutdown.is_set(),
                timeout=max(0.05, float(timeout)))
            return self.snapshot()

    def _update_debug_params(self, incoming: Dict[str, Any]) -> Dict[str, Any]:
        errors = []
        updated = {}
        specs = {
            "map_image_enabled": ("bool", None),
            "map_show_grid": ("bool", None),
            "map_show_labels": ("bool", None),
            "trail_enabled": ("bool", None),
            "show_last_known": ("bool", None),
            "trail_max_points": ("int", (5, 1000)),
            "robot_marker_radius_px": ("int", (4, 60)),
            "air_marker_radius_px": ("int", (4, 60)),
            "stale_timeout_s": ("float", (0.2, 30.0)),
            "detection_keep_count": ("int", (1, 500)),
            "wave_max_samples": ("int", (10, 20000)),
            "wave_history_keep": ("int", (1, 200)),
        }
        with self._lock:
            for key, value in incoming.items():
                if key not in specs:
                    errors.append(f"未知参数: {key}")
                    continue
                kind, bounds = specs[key]
                try:
                    if kind == "bool":
                        if not isinstance(value, bool):
                            raise ValueError
                        new_value = value
                    elif kind == "int":
                        number = int(value)
                        low, high = bounds
                        new_value = max(low, min(high, number))
                    else:
                        number = float(value)
                        low, high = bounds
                        new_value = max(low, min(high, number))
                except (TypeError, ValueError):
                    errors.append(f"参数类型错误: {key}")
                    continue
                self.debug_params[key] = new_value
                updated[key] = new_value
            if updated:
                if "wave_history_keep" in updated:
                    self.wave_store.history = deque(self.wave_store.history,
                                                    maxlen=int(updated["wave_history_keep"]))
                if "trail_max_points" in updated:
                    trail_max = int(updated["trail_max_points"])
                    for entry in self.robot_states.values():
                        entry["trail"] = deque(entry["trail"], maxlen=trail_max)
                self._save_debug_params()
                self._notify_locked()
            return {"ok": not errors, "updated": updated, "errors": errors,
                    "debug": _jsonable(dict(self.debug_params))}


# --------------------------------------------------------------------------- #
# HTTP 服务
# --------------------------------------------------------------------------- #
class RadarWebUiHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, handler_class, app: RadarWebUiNode):
        super().__init__(address, handler_class)
        self.app = app


class WebUiHandler(BaseHTTPRequestHandler):
    server_version = "RMRadarWebUI/1.0"
    protocol_version = "HTTP/1.1"

    @property
    def app(self) -> RadarWebUiNode:
        return self.server.app  # type: ignore[attr-defined]

    # ----------------------------- 基础工具 ----------------------------- #
    def log_message(self, fmt: str, *args) -> None:  # noqa: A003
        try:
            message = fmt % args
        except Exception:
            message = fmt
        if not any(part in message for part in ("/api/events", "/stream/")):
            self.app._log("debug", f"HTTP {message}")

    def _send_json(self, data: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(_jsonable(data), ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache, no-store")
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self) -> bytes:
        try:
            length = int(self.headers.get("Content-Length", "0") or 0)
        except ValueError:
            length = 0
        if length <= 0:
            return b""
        if length > MAX_BODY_BYTES:
            raise ValueError("body 超过 8MB 限制")
        return self.rfile.read(length)

    def _read_json_body(self) -> Any:
        raw = self._read_body()
        if not raw:
            raise ValueError("请求体为空")
        return json.loads(raw.decode("utf-8"))

    def _send_error(self, status: HTTPStatus, message: str) -> None:
        self._send_json({"ok": False, "error": message}, status)

    def _query(self, name: str, default: str = "") -> str:
        parsed = urlparse(self.path)
        values = parse_qs(parsed.query).get(name, [])
        return values[0] if values else default

    # ----------------------------- 路由 ----------------------------- #
    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path
        try:
            if path == "/" or path == "/index.html":
                self._send_static("/index.html")
                return
            if path == "/api/state":
                self._send_json(self.app.snapshot())
                return
            if path == "/api/health":
                snapshot = self.app.snapshot()
                self._send_json({"ok": True, "uptime": snapshot["node"]["uptime"],
                                 "generation": snapshot["generation"]})
                return
            if path == "/api/events":
                self._send_events()
                return
            if path == "/api/wave":
                self._send_json(self.app.wave_store.snapshot())
                return
            if path == "/api/wave/schema":
                self._send_json(WAVE_SCHEMA)
                return
            if path == "/api/debug":
                self._send_json({"ok": True, "debug": _jsonable(dict(self.app.debug_params)),
                                 "defaults": _jsonable(dict(DEFAULT_DEBUG_PARAMS))})
                return
            if path == "/api/map.png":
                self._send_map_image()
                return
            if path == "/stream/map_view":
                self._send_mjpeg("map_view")
                return
            if path == "/stream/detect_view":
                self._send_mjpeg("detect_view")
                return
            if path.startswith("/static/") or path.startswith("/assets/"):
                self._send_static(path)
                return
            if path.startswith("/api/"):
                self._send_error(HTTPStatus.NOT_FOUND, f"未知接口: {path}")
                return
            self._send_static(path)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return
        except Exception as exc:
            try:
                self._send_error(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))
            except Exception:
                pass

    def do_POST(self) -> None:  # noqa: N802
        self._handle_mutation("POST")

    def do_PUT(self) -> None:  # noqa: N802
        self._handle_mutation("PUT")

    def _handle_mutation(self, method: str) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        try:
            if path == "/api/wave/ingest":
                payload = self._read_json_body()
                byte_count = int(self.headers.get("Content-Length", "0") or 0)
                result = self.app.wave_store.ingest(payload, "http:" + path, byte_count=byte_count)
                with self.app._lock:
                    self.app._topic_touch("wave_http_ingest", "http/json", _now(),
                                          extra={"sample_count": result.get("sample_count", 0)})
                    self.app._log("info" if result.get("ok") else "warn",
                                  "HTTP 解析波数据" + ("已接收" if result.get("ok") else "被拒绝"),
                                  result)
                    self.app._notify_locked()
                self._send_json(result, HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST)
                return
            if path == "/api/wave/clear":
                result = self.app.wave_store.clear()
                with self.app._lock:
                    self.app._notify_locked()
                self._send_json(result)
                return
            if path == "/api/debug":
                payload = self._read_json_body()
                if not isinstance(payload, dict):
                    self._send_error(HTTPStatus.BAD_REQUEST, "参数必须是 JSON object")
                    return
                result = self.app._update_debug_params(payload)
                self._send_json(result, HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST)
                return
            self._send_error(HTTPStatus.NOT_FOUND, f"未知接口: {path}")
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return
        except ValueError as exc:
            try:
                self._send_error(HTTPStatus.BAD_REQUEST, str(exc))
            except Exception:
                pass
        except Exception as exc:
            try:
                self._send_error(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))
            except Exception:
                pass

    # ----------------------------- 静态文件 ----------------------------- #
    def _safe_static_path(self, url_path: str) -> Path:
        relative = url_path.lstrip("/")
        if not relative:
            relative = "index.html"
        if relative.startswith("static/"):
            relative = relative[len("static/"):]
        elif relative.startswith("assets/"):
            relative = relative[len("assets/"):]
        # 注意: symlink-install 下 web_root 内文件是软链接,不能 resolve(),
        # 否则 candidate 会跳到源目录导致 commonpath 检查误报。
        root = os.path.abspath(str(self.app.web_root))
        candidate = os.path.abspath(os.path.join(root, *relative.split("/")))
        if os.path.commonpath([root, candidate]) != root:
            raise ValueError("非法路径")
        return Path(candidate)

    def _send_static(self, url_path: str) -> None:
        path = self._safe_static_path(url_path)
        if path.is_dir() or not path.is_file():
            self._send_error(HTTPStatus.NOT_FOUND, f"文件不存在: {url_path}")
            return
        content_type = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
        if content_type.startswith("text/") or content_type in ("application/javascript", "image/svg+xml"):
            content_type += "; charset=utf-8"
        body = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache, no-store")
        self.end_headers()
        self.wfile.write(body)

    # ----------------------------- 地图图片 ----------------------------- #
    def _send_map_image(self) -> None:
        path = self.app._resolve_relative_path(
            str((self.app.config_summary.get("scene") or {}).get("std_map") or ""))
        if path is None:
            self._send_error(HTTPStatus.NOT_FOUND, "地图图片不存在, 请检查 main_config.yaml scene.std_map")
            return
        body = path.read_bytes()
        content_type = mimetypes.guess_type(str(path))[0] or "image/png"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    # ----------------------------- SSE ----------------------------- #
    def _send_events(self) -> None:
        last_generation = 0
        try:
            last_generation = int(float(self._query("generation", "0") or 0))
        except ValueError:
            last_generation = 0
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache, no-store")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        try:
            self.wfile.write(b"retry: 1000\n\n")
            self.wfile.flush()
        except Exception:
            return

        with self.app._lock:
            self.app.sse_clients.add(id(self))
        last_send = 0.0
        try:
            payload = self.app.snapshot()
            while not self.app._shutdown.is_set():
                generation = int(payload.get("generation", 0))
                now = float(payload.get("now", _now()))
                if generation > last_generation:
                    # 最多 10Hz 推送,降低多客户端/大波形的带宽压力
                    if last_generation == 0 or now - last_send >= 0.1:
                        body = json.dumps(_jsonable(payload), ensure_ascii=False,
                                          separators=(",", ":")).encode("utf-8")
                        self.wfile.write(b"data: " + body + b"\n\n")
                        self.wfile.flush()
                        last_generation = generation
                        last_send = now
                    else:
                        time.sleep(0.02)
                        payload = self.app.snapshot()
                else:
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
                    payload = self.app.wait_snapshot(last_generation, timeout=SSE_KEEPALIVE_SEC)
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            pass
        finally:
            with self.app._lock:
                self.app.sse_clients.discard(id(self))

    # ----------------------------- MJPEG ----------------------------- #
    def _send_mjpeg(self, source: str) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", 'multipart/x-mixed-replace; boundary="rmradar-frame"')
        self.send_header("Cache-Control", "no-cache, no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        boundary = b"--rmradar-frame\r\n"
        last_sequence = -1
        try:
            while not self.app._shutdown.is_set():
                with self.app._lock:
                    frame = dict(self.app.image_frames.get(source) or {})
                    sequence = int(self.app.image_sequence.get(source, 0))
                if frame and "jpeg_bytes" in frame and sequence != last_sequence:
                    body = boundary
                    body += b"Content-Type: image/jpeg\r\n"
                    body += f"Content-Length: {len(frame['jpeg_bytes'])}\r\n".encode("ascii")
                    body += b"\r\n"
                    body += frame["jpeg_bytes"]
                    body += b"\r\n"
                    self.wfile.write(body)
                    self.wfile.flush()
                    last_sequence = sequence
                time.sleep(MJPEG_INTERVAL_SEC)
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            return


# --------------------------------------------------------------------------- #
# 主入口
# --------------------------------------------------------------------------- #
def main(argv: Optional[List[str]] = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)

    # 解析 Web UI 自己的参数, ROS 参数仍通过 --ros-args 传递
    parser = argparse.ArgumentParser(description="RM_radar_Cpp_2027 调试 Web UI")
    parser.add_argument("--host", default=os.environ.get("RADAR_WEB_HOST", DEFAULT_HOST))
    parser.add_argument("--port", type=int,
                        default=int(os.environ.get("RADAR_WEB_PORT", str(DEFAULT_PORT))))
    parser.add_argument("--config", default="configs/main_config.yaml")
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = RadarWebUiNode(host=args.host, port=args.port, config_path=args.config)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    server = RadarWebUiHttpServer((node.host, node.port), WebUiHandler, node)
    server_thread = threading.Thread(target=server.serve_forever, name="radar-web-ui-http", daemon=True)

    def shutdown(signum=None, frame=None) -> None:  # noqa: ANN001
        node._shutdown.set()
        with node._lock:
            node._notify_locked()
        # 触发 main 里的 except KeyboardInterrupt,进入优雅关闭流程
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        server_thread.start()
        node.get_logger().info(f"Web UI 已启动: http://{node.host}:{node.port} (Ctrl+C 退出)")
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node._shutdown.set()
        server.shutdown()
        server.server_close()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
