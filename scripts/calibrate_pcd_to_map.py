#!/usr/bin/env python3
"""
calibrate_pcd_to_map.py — 3D PCD → 2D Map 变换矩阵交互式标定工具 (PyQt5)

参照 perspective_calibrator 的双视图点选模式：
  - 左侧：3D PCD 俯视投影（XY），可切换 Z 高度层（类似地面层/高地层）
  - 右侧：2D 场地地图
  - 左右依次点击对应的特征点对
  - 计算 Homography + 刚性变换矩阵
  - 保存对齐后的 PCD 及标定参数 JSON

操作流程：
  1. 确认 Z 高度层显示（默认地面层，可切换查看不同高度）
  2. 左图点击 PCD 俯视图特征点
  3. 右图点击 2D 地图对应物理位置
  4. 至少标定 4 对点（RANSAC 最少 4 对）
  5. 点击「计算并保存」写入标定文件

入口：
  python3 scripts/calibrate_pcd_to_map.py [--scene lab]

输出：
  - configs/calibration/pcd_to_map_calib.json   — 标定参数
  - source/pointclouds/background/background2_aligned.pcd — 变换后 PCD
"""

import sys
import os
import json
import struct
import re
import time
from pathlib import Path

# ── 解决 PyQt5 与 OpenCV 内置 Qt 插件冲突 ──
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
import cv2
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage, QTextCursor
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QTextEdit, QGridLayout,
)

# ======================== 路径配置 ========================
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent

MAIN_CONFIG_PATH = PROJECT_ROOT / "configs" / "main_config.yaml"
CALIB_OUTPUT_DIR = PROJECT_ROOT / "configs" / "calibration"
CALIB_JSON_PATH = CALIB_OUTPUT_DIR / "pcd_to_map_calib.json"
ALIGNED_PCD_PATH = PROJECT_ROOT / "source" / "pointclouds" / "background" / "background2_aligned.pcd"

# 显示面板尺寸
PCD_DISPLAY_W = 700
PCD_DISPLAY_H = 450
MAP_DISPLAY_W = 840
MAP_DISPLAY_H = 450

# PCD 俯视图生成时随机采样上限
PCD_SAMPLE_MAX = 50000

# Step 步长
STEP_Z = 0.5       # Z 层步长 (m)


# ======================== 配置加载 ========================
def load_config():
    try:
        from ruamel.yaml import YAML
        yaml = YAML()
        with open(MAIN_CONFIG_PATH, encoding="utf-8") as f:
            return yaml.load(f) or {}
    except Exception as e:
        print(f"[WARN] 无法加载 main_config.yaml: {e}")
        return {}


def get_scene_paths(cfg):
    scene = cfg.get("global", {}).get("scene", "lab")
    scene_cfg = cfg.get("scenes", {}).get(scene, {})
    return {
        "scene": scene,
        "std_map": str(PROJECT_ROOT / scene_cfg.get(
            "std_map", "source/maps/lab/map_2d_cropped.png")),
        "pcd_file": str(PROJECT_ROOT / scene_cfg.get(
            "pcd_file", "source/pointclouds/background/background2.pcd")),
        "field_w": float(scene_cfg.get("field_width", 28.0)),
        "field_h": float(scene_cfg.get("field_height", 15.0)),
    }


# ======================== PCD 读取 / 写入 ========================
def load_pcd(path):
    """读取 binary/ascii PCD，返回 (N,3) float32 数组和 header 字符串"""
    with open(path, "rb") as f:
        raw = f.read()

    header_end = raw.find(b"DATA binary")
    is_ascii = (header_end == -1)
    if is_ascii:
        header_end = raw.find(b"DATA ascii")

    header = raw[:header_end].decode("utf-8", errors="replace")
    pts_count = int(re.search(r"POINTS (\d+)", header).group(1))
    fields = re.search(r"FIELDS (.*)", header).group(1).split()
    sizes = [int(x) for x in re.search(r"SIZE (.*)", header).group(1).split()]
    data_start = raw.find(b"\n", header_end) + 1
    data = raw[data_start:]

    xyz = np.zeros((pts_count, 3), dtype=np.float32)

    if is_ascii:
        text = data.decode("utf-8", errors="replace")
        for i, line in enumerate(text.strip().split("\n")):
            if i >= pts_count: break
            parts = line.split()
            if len(parts) >= 3:
                xyz[i] = [float(parts[0]), float(parts[1]), float(parts[2])]
    else:
        pt_sz = sum(sizes)
        xi = fields.index("x") * 4
        yi = fields.index("y") * 4
        zi = fields.index("z") * 4
        for i in range(pts_count):
            off = i * pt_sz
            xyz[i] = [
                struct.unpack("f", data[off+xi:off+xi+4])[0],
                struct.unpack("f", data[off+yi:off+yi+4])[0],
                struct.unpack("f", data[off+zi:off+zi+4])[0],
            ]

    return xyz, header


def write_pcd(path, xyz, header):
    pts_count = len(xyz)
    new_header = re.sub(r"POINTS \d+", f"POINTS {pts_count}", header)
    new_header = re.sub(r"WIDTH \d+", f"WIDTH {pts_count}", new_header)
    if not new_header.endswith("\n"):
        new_header += "\n"
    with open(path, "wb") as f:
        f.write(new_header.encode("utf-8"))
        f.write(b"DATA binary\n")
        for i in range(pts_count):
            f.write(struct.pack("fff", xyz[i,0], xyz[i,1], xyz[i,2]))


# ======================== PCD 俯视图生成 ========================
def make_pcd_topview(xyz, z_min, z_max, w, h):
    """
    将 3D PCD 投影为俯视 (XY) 彩色图像。
    Z 低→蓝，Z 高→红。
    返回 (BGR图像, bounds_dict)
    """
    mask = (xyz[:,2] >= z_min) & (xyz[:,2] <= z_max)
    proj = xyz[mask]
    if len(proj) > PCD_SAMPLE_MAX:
        idx = np.random.choice(len(proj), PCD_SAMPLE_MAX, replace=False)
        proj = proj[idx]

    if len(proj) == 0:
        img = np.zeros((h, w, 3), dtype=np.uint8)
        return img, {"x_min": 0, "x_max": 1, "y_min": 0, "y_max": 1}

    xs, ys = proj[:,0], proj[:,1]
    x_min, x_max = float(xs.min()), float(xs.max())
    y_min, y_max = float(ys.min()), float(ys.max())

    # 留 5% 边距，保持纵横比
    m = 0.05
    rx = x_max - x_min + 1e-6
    ry = y_max - y_min + 1e-6
    x_min -= rx * m; x_max += rx * m
    y_min -= ry * m; y_max += ry * m
    rx = x_max - x_min; ry = y_max - y_min

    aspect = w / h
    if rx / ry > aspect:
        new_ry = rx / aspect
        mid = (y_min + y_max) / 2
        y_min = mid - new_ry/2; y_max = mid + new_ry/2
    else:
        new_rx = ry * aspect
        mid = (x_min + x_max) / 2
        x_min = mid - new_rx/2; x_max = mid + new_rx/2

    bounds = {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max}

    # 逐点绘制（叠加着色）
    img = np.zeros((h, w, 3), dtype=np.uint8)
    rng_x = x_max - x_min
    rng_y = y_max - y_min
    rng_z = max(z_max - z_min, 1e-6)

    for pt in proj:
        px = int((pt[0] - x_min) / rng_x * (w-1))
        py = int(h - 1 - (pt[1] - y_min) / rng_y * (h-1))
        if not (0 <= px < w and 0 <= py < h):
            continue
        zn = np.clip((pt[2] - z_min) / rng_z, 0, 1)
        b = int(255*(1-zn)); r = int(255*zn); g = int(100*(1-abs(zn-0.5)*2))
        old = img[py, px].astype(np.int32)
        img[py, px] = [min(255, old[0]+b), min(255, old[1]+g), min(255, old[2]+r)]

    return img, bounds


# ======================== 坐标转换工具 ========================
def pcd_disp_to_world(px, py, bounds, dw, dh):
    """PCD 显示像素 → PCD 世界坐标 (wx, wy)"""
    x = bounds["x_min"] + (px/dw) * (bounds["x_max"] - bounds["x_min"])
    y = bounds["y_max"] - (py/dh) * (bounds["y_max"] - bounds["y_min"])
    return x, y


def pcd_world_to_disp(wx, wy, bounds, dw, dh):
    """PCD 世界坐标 → PCD 显示像素 (px, py)"""
    px = int((wx - bounds["x_min"]) / (bounds["x_max"] - bounds["x_min"]) * (dw-1))
    py = int(dh - 1 - (wy - bounds["y_min"]) / (bounds["y_max"] - bounds["y_min"]) * (dh-1))
    return px, py


def map_disp_to_world(px, py, mw, mh, fw, fh, is_portrait, dw, dh):
    """2D 地图显示像素 → 赛场世界坐标 (wx, wy)"""
    # 显示像素 → 原图像素
    sx = mw / dw
    sy = mh / dh
    mx = px * sx
    my = py * sy
    if is_portrait:
        wx = my / mh * fw
        wy = (mw - mx) / mw * fh
    else:
        wx = mx / mw * fw
        wy = (mh - my) / mh * fh
    return wx, wy


def map_world_to_disp(wx, wy, mw, mh, fw, fh, is_portrait, dw, dh):
    """赛场世界坐标 → 2D 地图显示像素"""
    if is_portrait:
        mx = (fw - wy) / fw * mw   # hmm this might not be right
        my = wx / fw * mh
        # Let me re-derive. is_portrait: mh>mw. H→X(28m), W→Y(15m)
        # world wx → map pixel my: my/mh = wx/fw → my = wx/fw * mh
        # world wy → map pixel mx: mx/mw = 1 - wy/fh → mx = (1 - wy/fh) * mw
        my = wx / fw * mh
        mx = (1.0 - wy / fh) * mw
    else:
        mx = wx / fw * mw
        my = mh - wy / fh * mh
    # 原图像素 → 显示像素
    px = int(mx / (mw / dw))
    py = int(my / (mh / dh))
    return px, py


# ======================== Z 层颜色（与 perspective_calibrator 高度层类比） ========================
Z_PRESETS = {
    "全部 (-2~3m)":     (-2.0, 3.0),
    "地面层 (-0.5~0.5m)": (-0.5, 0.5),
    "低层 (0~1m)":      (0.0, 1.0),
    "中层 (0.5~1.5m)":  (0.5, 1.5),
    "高层 (1~2m)":      (1.0, 2.0),
    "屋顶 (2~3m)":      (2.0, 3.0),
}


# ======================== 主 UI ========================
class PcdMapCalibrator(QWidget):
    """3D PCD ↔ 2D Map 变换标定器 — 参照 perspective_calibrator 结构"""

    def __init__(self, scene="lab"):
        super().__init__()

        # ── 加载配置 ──
        cfg = load_config()
        paths = get_scene_paths(cfg)
        self.scene = paths["scene"]
        self.map_path = paths["std_map"]
        self.pcd_path = paths["pcd_file"]
        self.field_w = paths["field_w"]
        self.field_h = paths["field_h"]
        self.my_color = cfg.get("global", {}).get("my_color", "Blue")

        # ── 标定点数据 ──
        # 参照 perspective_calibrator，存储像素坐标
        # pcd_points: [(px, py)] 在 PCD 俯视图原图分辨率下的像素
        # map_points:  [(px, py)] 在地图原图分辨率下的像素
        self.pcd_points = []
        self.map_points = []

        # ── Z 层状态 ──
        self.z_min = -0.5
        self.z_max = 0.5

        # ── PCD 数据 ──
        self.pcd_xyz = None        # (N,3) float32
        self.pcd_header = None     # str
        self.pcd_bounds = None     # dict 当前俯视图的坐标边界

        # ── 地图数据 ──
        self.map_orig = None       # BGR 原图
        self.map_w_orig = 2800
        self.map_h_orig = 1500
        self.is_portrait = False

        # ── 显示缩放比（为匹配 perspective_calibrator 的 left_scale / right_scale 概念） ──
        # PCD 侧：俯视图渲染为 PCD_DISPLAY_W × PCD_DISPLAY_H，scale = 1（显示即原图）
        self.left_scale_x = 1.0
        self.left_scale_y = 1.0
        # 地图侧：原图缩放至 MAP_DISPLAY_W × MAP_DISPLAY_H
        self.right_scale_x = 1.0
        self.right_scale_y = 1.0

        # ── 缓存显示图像 ──
        self._pcd_display = None
        self._map_display = None

        # ── 初始化数据 ──
        self._init_data()

        # ── 构建 UI ──
        self._init_ui()

    # ================================================================
    #  数据加载
    # ================================================================
    def _init_data(self):
        # PCD
        if os.path.exists(self.pcd_path):
            self.pcd_xyz, self.pcd_header = load_pcd(self.pcd_path)
            self._log(f"PCD: {len(self.pcd_xyz)} 点")
            self._log(f"  X [{self.pcd_xyz[:,0].min():.2f}, {self.pcd_xyz[:,0].max():.2f}] "
                      f"span={self.pcd_xyz[:,0].max()-self.pcd_xyz[:,0].min():.2f}m")
            self._log(f"  Y [{self.pcd_xyz[:,1].min():.2f}, {self.pcd_xyz[:,1].max():.2f}] "
                      f"span={self.pcd_xyz[:,1].max()-self.pcd_xyz[:,1].min():.2f}m")
            self._log(f"  Z [{self.pcd_xyz[:,2].min():.2f}, {self.pcd_xyz[:,2].max():.2f}] "
                      f"span={self.pcd_xyz[:,2].max()-self.pcd_xyz[:,2].min():.2f}m")
        else:
            self._log(f"⚠ PCD 不存在: {self.pcd_path}")

        # 地图
        if os.path.exists(self.map_path):
            self.map_orig = cv2.imread(self.map_path)
            self.map_h_orig, self.map_w_orig = self.map_orig.shape[:2]
            self.is_portrait = (self.map_h_orig > self.map_w_orig)
            self.right_scale_x = self.map_w_orig / MAP_DISPLAY_W
            self.right_scale_y = self.map_h_orig / MAP_DISPLAY_H
            self._log(f"地图: {self.map_w_orig}×{self.map_h_orig} "
                      f"({'竖版' if self.is_portrait else '横版'})")
        else:
            self._log(f"⚠ 地图不存在: {self.map_path}")

        self._log(f"场地: {self.field_w}×{self.field_h} m")
        self._log("─" * 40)

        # 尝试加载已有标定
        if CALIB_JSON_PATH.exists():
            try:
                with open(CALIB_JSON_PATH) as f:
                    prev = json.load(f)
                self._log(f"ℹ 已有标定: dx={prev.get('dx',0):.3f}, "
                          f"dy={prev.get('dy',0):.3f}, "
                          f"θ={prev.get('theta_deg',0):.2f}°")
            except Exception:
                pass

    # ================================================================
    #  UI 构建 — 参照 perspective_calibrator._init_ui 布局
    # ================================================================
    def _init_ui(self):
        # ── 左侧：PCD 俯视图 ──
        self.left_label = QLabel(self)
        self.left_label.setFixedSize(PCD_DISPLAY_W, PCD_DISPLAY_H)
        self.left_label.setStyleSheet("border: 2px solid #666; background: #0a0a0a;")
        self.left_label.mousePressEvent = self._left_clicked

        # ── 右侧：2D 地图 ──
        self.right_label = QLabel(self)
        self.right_label.setFixedSize(MAP_DISPLAY_W, MAP_DISPLAY_H)
        self.right_label.setStyleSheet("border: 2px solid #666;")
        self.right_label.mousePressEvent = self._right_clicked

        # ── Z 层切换按钮（类比 perspective_calibrator 的 btn_switch） ──
        self.z_buttons = {}
        z_btn_layout = QHBoxLayout()
        z_btn_layout.addWidget(QLabel("Z 层:"))
        for name in Z_PRESETS:
            btn = QPushButton(name.split()[0], self)  # 只显示第一个词
            btn.setCheckable(True)
            btn.setFixedSize(60, 28)
            btn.clicked.connect(lambda checked, n=name: self._on_z_preset(n))
            self.z_buttons[name] = btn
            z_btn_layout.addWidget(btn)
        # 默认选中「地面层」
        default_z = "地面层 (-0.5~0.5m)"
        if default_z in self.z_buttons:
            self.z_buttons[default_z].setChecked(True)
            self.z_buttons[default_z].setStyleSheet(
                "background: #28a; color: white; font-weight: bold;")
        z_btn_layout.addStretch()

        # Z 微调
        self.z_info_label = QLabel(f"Z: {self.z_min:.1f}~{self.z_max:.1f}m")
        self.z_info_label.setFixedWidth(140)
        z_btn_layout.addWidget(self.z_info_label)

        # ── 按钮 — 参照 perspective_calibrator 的按钮命名 ──
        self.btn_undo = QPushButton("撤销上一点", self)
        self.btn_undo.setFixedSize(120, 36)
        self.btn_undo.clicked.connect(self._on_undo)

        self.btn_reset = QPushButton("重置所有点", self)
        self.btn_reset.setFixedSize(120, 36)
        self.btn_reset.clicked.connect(self._on_reset)

        self.btn_save = QPushButton("保存计算", self)
        self.btn_save.setFixedSize(130, 40)
        self.btn_save.setStyleSheet(
            "QPushButton { background: #2a8; color: white; font-weight: bold; "
            "font-size: 14px; border-radius: 5px; }"
            "QPushButton:hover { background: #3b9; }")
        self.btn_save.clicked.connect(self._on_save)

        # ── 状态栏 ──
        self.status_label = QLabel(self)
        self.status_label.setFixedSize(MAP_DISPLAY_W, 28)
        self._update_status()

        # ── 日志 ──
        self.text_edit = QTextEdit(self)
        self.text_edit.setFixedSize(MAP_DISPLAY_W, 150)
        self.text_edit.setReadOnly(True)

        # ── 布局 — 同 perspective_calibrator ──
        btn_grid = QGridLayout()
        btn_grid.addWidget(self.btn_undo, 0, 0)
        btn_grid.addWidget(self.btn_reset, 0, 1)
        btn_grid.addWidget(self.btn_save, 1, 0, 1, 2)

        right_vbox = QVBoxLayout()
        right_vbox.addWidget(self.right_label)
        right_vbox.addLayout(z_btn_layout)
        right_vbox.addWidget(self.status_label)
        right_vbox.addLayout(btn_grid)
        right_vbox.addWidget(self.text_edit)

        hbox = QHBoxLayout()
        hbox.addWidget(self.left_label)
        hbox.addLayout(right_vbox)

        self.setLayout(hbox)
        self.setWindowTitle(
            f"3D PCD → 2D Map 变换标定 — "
            f"{self.my_color}方 — {self.scene}")
        self.setGeometry(30, 30, PCD_DISPLAY_W + MAP_DISPLAY_W + 40,
                         max(PCD_DISPLAY_H, MAP_DISPLAY_H + 260))

        # ── 初始刷新 ──
        self._refresh_both()
        self.show()

    # ================================================================
    #  绘制 — 参照 perspective_calibrator._refresh_images
    # ================================================================
    def _refresh_both(self):
        # ── 左：PCD 俯视图 ──
        if self.pcd_xyz is not None:
            pcd_img, self.pcd_bounds = make_pcd_topview(
                self.pcd_xyz, self.z_min, self.z_max,
                PCD_DISPLAY_W, PCD_DISPLAY_H)
        else:
            pcd_img = np.zeros((PCD_DISPLAY_H, PCD_DISPLAY_W, 3), dtype=np.uint8)

        # 画 PCD 侧已选点
        for i, (px, py) in enumerate(self.pcd_points):
            cv2.circle(pcd_img, (px, py), 6, (0, 220, 255), -1)
            cv2.circle(pcd_img, (px, py), 8, (255, 255, 255), 2)
            cv2.putText(pcd_img, f"P{i+1}", (px+10, py-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 2)

        # 坐标范围标注
        if self.pcd_bounds:
            b = self.pcd_bounds
            cv2.putText(pcd_img,
                        f"X:[{b['x_min']:.1f},{b['x_max']:.1f}] "
                        f"Y:[{b['y_min']:.1f},{b['y_max']:.1f}]",
                        (5, PCD_DISPLAY_H-8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (140,140,140), 1)

        self._pcd_display = pcd_img
        self.left_label.setPixmap(self._cv_to_pixmap(pcd_img))

        # ── 右：地图 ──
        if self.map_orig is not None:
            map_show = cv2.resize(self.map_orig, (MAP_DISPLAY_W, MAP_DISPLAY_H))
        else:
            map_show = np.zeros((MAP_DISPLAY_H, MAP_DISPLAY_W, 3), dtype=np.uint8)

        # 网格
        sx_m = MAP_DISPLAY_W / self.field_w
        sy_m = MAP_DISPLAY_H / self.field_h
        for xm in range(int(self.field_w)+1):
            px = int(xm * sx_m)
            c = (0, 200, 0) if xm % 5 == 0 else (50, 50, 50)
            cv2.line(map_show, (px, 0), (px, MAP_DISPLAY_H), c, 1)
        for ym in range(int(self.field_h)+1):
            py = int(MAP_DISPLAY_H - ym * sy_m)
            c = (0, 200, 0) if ym % 5 == 0 else (50, 50, 50)
            cv2.line(map_show, (0, py), (MAP_DISPLAY_W, py), c, 1)
        # 坐标数字
        for xm in range(0, int(self.field_w)+1, 5):
            cv2.putText(map_show, str(xm), (int(xm*sx_m)+2, MAP_DISPLAY_H-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,200,0), 1)
        for ym in range(0, int(self.field_h)+1, 5):
            cv2.putText(map_show, str(ym), (2, int(MAP_DISPLAY_H-ym*sy_m)-2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,200,0), 1)
        # 原点
        cv2.circle(map_show, (0, MAP_DISPLAY_H-1), 8, (0,0,255), -1)
        cv2.putText(map_show, "O(0,0)", (8, MAP_DISPLAY_H-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

        # 地图侧已选点（转换为显示像素）
        for i, (mx, my) in enumerate(self.map_points):
            dpx = int(mx / self.right_scale_x)
            dpy = int(my / self.right_scale_y)
            cv2.circle(map_show, (dpx, dpy), 6, (100, 255, 0), -1)
            cv2.circle(map_show, (dpx, dpy), 8, (255, 255, 255), 2)
            cv2.putText(map_show, f"M{i+1}", (dpx+10, dpy-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 0), 2)

        self._map_display = map_show
        self.right_label.setPixmap(self._cv_to_pixmap(map_show))

    # ================================================================
    #  鼠标事件 — 参照 perspective_calibrator._left_clicked / _right_clicked
    # ================================================================
    def _left_clicked(self, event):
        """PCD 俯视图点击 — 记录显示像素坐标"""
        px = event.pos().x()
        py = event.pos().y()
        self.pcd_points.append((px, py))

        # 也计算世界坐标用于日志
        if self.pcd_bounds:
            wx, wy = pcd_disp_to_world(px, py, self.pcd_bounds,
                                       PCD_DISPLAY_W, PCD_DISPLAY_H)
            self._log(f"  PCD点 P{len(self.pcd_points)}: "
                      f"显示({px},{py}) → 世界({wx:.3f}, {wy:.3f})")
        else:
            self._log(f"  PCD点 P{len(self.pcd_points)}: 显示({px},{py})")

        self._update_status()
        self._refresh_both()

    def _right_clicked(self, event):
        """2D 地图点击 — ★ 参照 perspective_calibrator：存储地图像素坐标 ★"""
        dpx = event.pos().x()   # 显示像素
        dpy = event.pos().y()
        # 显示像素 → 原图像素
        map_px = int(dpx * self.right_scale_x)
        map_py = int(dpy * self.right_scale_y)
        self.map_points.append((map_px, map_py))

        # 日志中显示对应的世界坐标
        wx, wy = map_disp_to_world(dpx, dpy,
                                   self.map_w_orig, self.map_h_orig,
                                   self.field_w, self.field_h,
                                   self.is_portrait,
                                   MAP_DISPLAY_W, MAP_DISPLAY_H)
        self._log(f"  地图点 M{len(self.map_points)}: "
                  f"地图像素({map_px},{map_py}) → 世界({wx:.3f}, {wy:.3f})")

        self._update_status()
        self._refresh_both()

    # ================================================================
    #  按钮回调 — 参照 perspective_calibrator 按钮命名
    # ================================================================
    def _on_z_preset(self, name):
        self.z_min, self.z_max = Z_PRESETS[name]
        self.z_info_label.setText(f"Z: {self.z_min:.1f}~{self.z_max:.1f}m")
        # 更新按钮样式
        for n, btn in self.z_buttons.items():
            if n == name:
                btn.setChecked(True)
                btn.setStyleSheet("background: #28a; color: white; font-weight: bold;")
            else:
                btn.setChecked(False)
                btn.setStyleSheet("")
        self._refresh_both()

    def _on_undo(self):
        if self.map_points:
            self.map_points.pop()
        if self.pcd_points:
            self.pcd_points.pop()
        n = len(self.pcd_points)
        self._log(f"  撤销 → 剩余 {n} 对")
        self._update_status()
        self._refresh_both()

    def _on_reset(self):
        self.pcd_points.clear()
        self.map_points.clear()
        self._log("  已重置所有点")
        self._update_status()
        self._refresh_both()

    def _on_save(self):
        """计算 Homography + 刚性变换，保存 PCD 和 JSON"""
        n = min(len(self.pcd_points), len(self.map_points))
        if n < 4:
            self._log(f"✘ 至少需要 4 对点（RANSAC），当前 {n} 对")
            return

        # ── 1. PCD 显示像素 → PCD 世界坐标 ──
        pcd_world = []
        for px, py in self.pcd_points[:n]:
            wx, wy = pcd_disp_to_world(px, py, self.pcd_bounds,
                                       PCD_DISPLAY_W, PCD_DISPLAY_H)
            pcd_world.append([wx, wy])
        pcd_world = np.array(pcd_world, dtype=np.float64)

        # ── 2. 地图原图像素 → 地图世界坐标 ──
        map_world = []
        for mx, my in self.map_points[:n]:
            # 原图像素 → 显示像素 → 世界坐标
            dpx = mx / self.right_scale_x
            dpy = my / self.right_scale_y
            wx, wy = map_disp_to_world(dpx, dpy,
                                       self.map_w_orig, self.map_h_orig,
                                       self.field_w, self.field_h,
                                       self.is_portrait,
                                       MAP_DISPLAY_W, MAP_DISPLAY_H)
            map_world.append([wx, wy])
        map_world = np.array(map_world, dtype=np.float64)

        # ── 3. 计算相似变换 (4 DOF) — PCD world → Map world ──
        M, inliers = cv2.estimateAffinePartial2D(
            pcd_world.astype(np.float32),
            map_world.astype(np.float32),
        )
        if M is None:
            self._log("✘ 变换矩阵计算失败，请检查点对")
            return

        tx, ty = M[0,2], M[1,2]
        scale = np.sqrt(M[0,0]**2 + M[1,0]**2)
        theta_rad = np.arctan2(M[1,0], M[0,0])
        theta_deg = np.degrees(theta_rad)

        # 重投影误差
        errors = []
        for i in range(n):
            pred = M @ np.array([pcd_world[i][0], pcd_world[i][1], 1.0])
            err = np.linalg.norm(pred - map_world[i])
            errors.append(err)

        mean_err = np.mean(errors)
        max_err = np.max(errors)
        inl_cnt = int(np.sum(inliers)) if inliers is not None else n

        # ── 4. 构建 4×4 矩阵 ──
        c, s = np.cos(theta_rad), np.sin(theta_rad)
        T = np.eye(4)
        T[0,0] = scale*c; T[0,1] = -scale*s; T[0,3] = tx
        T[1,0] = scale*s; T[1,1] =  scale*c; T[1,3] = ty

        # ── 5. 日志 ──
        self._log("─" * 50)
        self._log("✔ 标定完成")
        self._log(f"  平移: dx={tx:.4f} m, dy={ty:.4f} m")
        self._log(f"  旋转: θ={theta_deg:.4f}°")
        self._log(f"  缩放: s={scale:.6f}")
        self._log(f"  内点: {inl_cnt}/{n}")
        self._log(f"  误差: mean={mean_err:.4f}m, max={max_err:.4f}m")
        for i, e in enumerate(errors):
            self._log(f"    #{i+1}: {e:.4f}m")
        self._log("")
        self._log("  4×4 变换矩阵:")
        self._log(f"  [{T[0,0]:10.6f} {T[0,1]:10.6f} {T[0,2]:10.6f} {T[0,3]:10.4f}]")
        self._log(f"  [{T[1,0]:10.6f} {T[1,1]:10.6f} {T[1,2]:10.6f} {T[1,3]:10.4f}]")
        self._log(f"  [{T[2,0]:10.6f} {T[2,1]:10.6f} {T[2,2]:10.6f} {T[2,3]:10.4f}]")
        self._log(f"  [{T[3,0]:10.6f} {T[3,1]:10.6f} {T[3,2]:10.6f} {T[3,3]:10.4f}]")

        # ── 6. 变换 PCD 并保存 ──
        if self.pcd_xyz is not None and self.pcd_header is not None:
            xy = self.pcd_xyz[:,:2] * scale
            xy_rot = xy @ np.array([[c, s], [-s, c]])
            aligned = self.pcd_xyz.copy()
            aligned[:,0] = xy_rot[:,0] + tx
            aligned[:,1] = xy_rot[:,1] + ty

            write_pcd(str(ALIGNED_PCD_PATH), aligned, self.pcd_header)
            self._log(f"\n  ✓ 对齐后 PCD: {ALIGNED_PCD_PATH}")
            self._log(f"    变换后 X: [{aligned[:,0].min():.2f}, "
                      f"{aligned[:,0].max():.2f}]")
            self._log(f"    变换后 Y: [{aligned[:,1].min():.2f}, "
                      f"{aligned[:,1].max():.2f}]")

        # ── 7. 保存 JSON（参照 perspective_calib.json 格式） ──
        calib = {
            "calibration_time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "scene": self.scene,
            "my_color": self.my_color,
            "map_file": self.map_path,
            "map_w": self.map_w_orig,
            "map_h": self.map_h_orig,
            "map_is_portrait": self.is_portrait,
            "pcd_file": self.pcd_path,
            "output_pcd": str(ALIGNED_PCD_PATH),
            "field_w": self.field_w,
            "field_h": self.field_h,
            # 相似变换参数
            "dx": round(float(tx), 6),
            "dy": round(float(ty), 6),
            "theta_deg": round(float(theta_deg), 6),
            "theta_rad": round(float(theta_rad), 6),
            "scale": round(float(scale), 6),
            "transform_matrix_4x4": T.tolist(),
            "affine_matrix_2x3": M.tolist(),
            # 标定点（世界坐标）
            "pcd_points_world": [[float(p[0]), float(p[1])] for p in pcd_world],
            "map_points_world": [[float(p[0]), float(p[1])] for p in map_world],
            # 标定点（像素坐标）
            "pcd_points_pixel": [[int(p[0]), int(p[1])] for p in self.pcd_points[:n]],
            "map_points_pixel": [[int(p[0]), int(p[1])] for p in self.map_points[:n]],
            # 误差
            "num_points": n,
            "inliers": inl_cnt,
            "mean_error_m": round(float(mean_err), 6),
            "max_error_m": round(float(max_err), 6),
            "errors_m": [round(float(e), 6) for e in errors],
        }
        os.makedirs(str(CALIB_OUTPUT_DIR), exist_ok=True)
        with open(str(CALIB_JSON_PATH), "w") as f:
            json.dump(calib, f, indent=2, ensure_ascii=False)
        self._log(f"  ✓ 标定参数: {CALIB_JSON_PATH}")
        self._log("─" * 50)
        self._log("如需使用对齐后的 PCD，请修改 main_config.yaml:")
        self._log(f"  scenes.{self.scene}.pcd_file: "
                  f"\"source/pointclouds/background/background2_aligned.pcd\"")
        self._log(f"  scenes.{self.scene}.downsampled_pcd: "
                  f"\"source/pointclouds/background/background2_aligned.pcd\"")

    # ================================================================
    #  辅助方法
    # ================================================================
    def _update_status(self):
        n_pcd = len(self.pcd_points)
        n_map = len(self.map_points)
        n = min(n_pcd, n_map)
        need = max(0, 4 - n)
        text = (f"PCD点: {n_pcd} | 地图点: {n_map} | 有效对: {n}")
        text += f" | 还需 {need} 对" if need > 0 else " | ✔ 可保存"
        color = "green" if need == 0 else ("orange" if n > 0 else "#ccc")
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            f"font-size: 13px; font-weight: bold; color: {color}; "
            f"background: #1e1e1e; padding: 4px;")

    def _log(self, text: str):
        self.text_edit.append(text)
        cursor = self.text_edit.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.text_edit.setTextCursor(cursor)

    @staticmethod
    def _cv_to_pixmap(cv_bgr):
        rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        return QPixmap.fromImage(qimg)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()
        elif event.key() == Qt.Key_U:
            self._on_undo()


# ======================== 入口 — 参照 perspective_calibrator.main ========================
def main():
    import argparse
    parser = argparse.ArgumentParser(description="3D PCD → 2D Map 变换标定")
    parser.add_argument("--scene", default="lab",
                        choices=["lab", "competition"])
    args = parser.parse_args()

    app = QApplication(sys.argv)
    _ = PcdMapCalibrator(scene=args.scene)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
