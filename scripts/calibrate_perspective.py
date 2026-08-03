#!/usr/bin/env python3
"""
calibrate_perspective.py — 透视变换矩阵标定脚本

对标 perspective_calibrator.py，生成 configs/perspective_calib.json。

核心流程（与 perspective_calibrator 一致）：
  1. 根据 camera.mode 自动采集相机第一帧（海康/USB/视频文件/测试图片）
  2. 加载地图图像（根据 my_color 选择红/蓝方视角地图）
  3. matplotlib 双图展示，左键点击左图（相机），右键点击右图（地图）
  4. ★ 所有点存储为像素坐标（与 perspective_calibrator 完全一致）
  5. cv2.findHomography 计算 H 矩阵
  6. 保存 perspective_calib.json

操作：
  左键点击左图  → 相机特征点
  右键点击右图  → 地图对应位置
  T             → 切换地面层 / 高地层
  U             → 撤销上一点
  S             → 计算 H 并保存 perspective_calib.json
  Q / Esc       → 退出

入口：
  python3 scripts/calibrate_perspective.py [--scene lab] [--image override.jpg]
"""

import sys
import os
import json
import time
import threading
from pathlib import Path

import cv2
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton

# ======================== 路径 ========================
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent

MAIN_CONFIG_PATH = str(PROJECT_ROOT / "configs" / "main_config.yaml")
PERSPECTIVE_CALIB_PATH = str(PROJECT_ROOT / "configs" / "perspective_calib.json")
HKCAM_PATH = str(PROJECT_ROOT.parent / "HNURM-radar-2026" / "src" / "hnurm_radar" / "hnurm_radar")

# 显示尺寸
LEFT_W, LEFT_H = 800, 600
RIGHT_W, RIGHT_H = 840, 450

# 高度层
LAYER_NAMES = ["地面层", "高地层"]


# ======================== 配置 ========================
def load_config():
    try:
        from ruamel.yaml import YAML
        return YAML().load(open(MAIN_CONFIG_PATH, encoding="utf-8")) or {}
    except Exception:
        import yaml
        return yaml.safe_load(open(MAIN_CONFIG_PATH, encoding="utf-8")) or {}


def resolve(path_str):
    if os.path.isabs(path_str):
        return path_str
    return str(PROJECT_ROOT / path_str)


# ======================== 相机图像采集（同 perspective_calibrator） ========================
camera_image = None  # 全局


def _hik_thread():
    global camera_image
    try:
        sys.path.insert(0, HKCAM_PATH)
        from Camera.HKCam import HKCam
        cam = HKCam(0)
        print("[相机] 海康工业相机已连接, 等待第一帧...")
        while camera_image is None:
            frame = cam.getFrame()
            if frame is not None:
                camera_image = frame
                print(f"[相机] 获取第一帧: {camera_image.shape[1]}x{camera_image.shape[0]}")
    except Exception as e:
        print(f"[相机] 海康相机失败: {e}")


def _video_thread(source):
    global camera_image
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"[相机] 无法打开视频源: {source}")
        return
    print(f"[相机] 视频源已打开: {source}, 等待第一帧...")
    while camera_image is None:
        ret, img = cap.read()
        if ret:
            camera_image = img
            print(f"[相机] 获取第一帧: {camera_image.shape[1]}x{camera_image.shape[0]}")


def capture_camera_frame(cfg):
    """根据 main_config.yaml 的 camera.mode 采集第一帧，返回 BGR 图像。
    相机不可用时快速回退到 test_image 配置的图片。"""
    global camera_image
    camera_cfg = cfg.get("camera", {})
    mode = camera_cfg.get("mode", "test")

    # ── 尝试打开相机 ──
    camera_image = None

    if mode == "hik":
        # 先检查 HKCam 模块是否可导入，避免无意义等待
        try:
            sys.path.insert(0, HKCAM_PATH)
            from Camera.HKCam import HKCam
            t = threading.Thread(target=_hik_thread, daemon=True)
            t.start()
            for _ in range(50):  # 5 秒超时
                if camera_image is not None:
                    return camera_image
                time.sleep(0.1)
            print("[相机] 超时(5s)！海康相机无响应，回退到测试图片")
        except ImportError:
            print("[相机] HKCam 模块不可用，回退到测试图片")
        except Exception as e:
            print(f"[相机] 海康相机错误: {e}，回退到测试图片")

    elif mode == "video":
        source = camera_cfg.get("video_source", 0)
        if isinstance(source, str):
            source = resolve(source)
        t = threading.Thread(target=_video_thread, args=(source,), daemon=True)
        t.start()
        for _ in range(30):  # 3 秒超时
            if camera_image is not None:
                return camera_image
            time.sleep(0.1)
        print("[相机] 超时(3s)！视频源无响应，回退到测试图片")

    # ── 回退：测试图片（从 main_config.yaml 读取） ──
    test_img = camera_cfg.get("test_image", "")
    if test_img:
        test_path = resolve(test_img)
    else:
        # 尝试几个默认位置
        candidates = [
            str(PROJECT_ROOT / "test_resources" / "test1.jpg"),
            str(PROJECT_ROOT.parent / "HNURM-radar-2026" / "test_resources" / "test1.jpg"),
        ]
        test_path = None
        for c in candidates:
            if os.path.exists(c):
                test_path = c
                break
        if not test_path:
            raise FileNotFoundError(
                "没有 camera.test_image 配置，也没有找到默认测试图片。\n"
                "请用 --image 参数指定一张相机截图。")

    print(f"[图像] 使用图片: {test_path}")
    img = cv2.imread(test_path)
    if img is None:
        raise FileNotFoundError(f"无法读取图片: {test_path}")
    return img


# ======================== 主程序 ========================
def main():
    import argparse
    parser = argparse.ArgumentParser(description="透视变换矩阵标定 — perspective_calib.json")
    parser.add_argument("--scene", default=None)
    parser.add_argument("--image", default=None,
                        help="手动指定相机图像路径 (覆盖自动采集)")
    parser.add_argument("--output", default=None,
                        help="输出 JSON 路径")
    args = parser.parse_args()

    cfg = load_config()
    my_color = cfg.get("global", {}).get("my_color", "Blue")
    scene = args.scene or cfg.get("global", {}).get("scene", "lab")
    scene_cfg = cfg.get("scenes", {}).get(scene, {})
    field_w = float(scene_cfg.get("field_width", 28.0))
    field_h = float(scene_cfg.get("field_height", 15.0))
    output_path = args.output or PERSPECTIVE_CALIB_PATH

    # ── 1. 相机图像 ──
    if args.image:
        cam_img = cv2.imread(args.image)
        if cam_img is None:
            print(f"[ERROR] 无法读取: {args.image}")
            sys.exit(1)
        print(f"[图像] 手动指定: {args.image} ({cam_img.shape[1]}x{cam_img.shape[0]})")
    else:
        cam_img = capture_camera_frame(cfg)

    cam_h, cam_w = cam_img.shape[:2]

    # ── 2. 地图 ──
    if my_color == "Red":
        map_rel = scene_cfg.get("pfa_map_red", scene_cfg.get("pfa_map", ""))
    else:
        map_rel = scene_cfg.get("pfa_map_blue", scene_cfg.get("pfa_map", ""))
    if not map_rel:
        map_rel = scene_cfg.get("pfa_map",
                                "source/maps/lab/lab_map_28x15.png")
    map_path = resolve(map_rel)
    map_img = cv2.imread(map_path)
    if map_img is None:
        print(f"[ERROR] 无法读取地图: {map_path}")
        sys.exit(1)
    map_h_orig, map_w_orig = map_img.shape[:2]
    is_portrait = map_h_orig > map_w_orig

    print("=" * 60)
    print("  透视变换矩阵标定工具")
    print("=" * 60)
    print(f"  方阵: {my_color}  场景: {scene}")
    print(f"  相机: {cam_w}x{cam_h}")
    print(f"  地图: {map_w_orig}x{map_h_orig} "
          f"({'竖版' if is_portrait else '横版'})  场地: {field_w}x{field_h}m")
    print(f"  输出: {output_path}")
    print("=" * 60)
    print("  操作: 左键左图 | 右键右图 | T切换层 | U撤销 | S保存 | Q退出")
    print()

    # ── 3. 缩放比 ──
    cam_sx = cam_w / LEFT_W
    cam_sy = cam_h / LEFT_H
    map_sx = map_w_orig / RIGHT_W
    map_sy = map_h_orig / RIGHT_H

    # ── 4. 标定状态 ──
    image_points = [[], []]   # [地面层, 高地层] → 相机像素 [px, py]
    map_points = [[], []]     # [地面层, 高地层] → 地图像素 [px, py]
    current_layer = 0

    # ── 5. 绘图 ──
    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(16, 7))
    fig.canvas.manager.set_window_title(
        f"Perspective Calibration - {my_color} - perspective_calib.json")

    # 左图
    left_rgb = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)
    left_resized = cv2.resize(left_rgb, (LEFT_W, LEFT_H))
    ax_left.imshow(left_resized)
    ax_left.set_title(f"相机 ({cam_w}x{cam_h}) — {LAYER_NAMES[current_layer]}",
                      fontsize=11)
    (left_scatter,) = ax_left.plot([], [], "o", color="cyan", markersize=8,
                                     markeredgecolor="white", markeredgewidth=1.5)

    # 右图
    right_rgb = cv2.cvtColor(map_img, cv2.COLOR_BGR2RGB)
    right_resized = cv2.resize(right_rgb, (RIGHT_W, RIGHT_H))
    ax_right.imshow(right_resized)
    ax_right.set_title(f"地图 ({map_w_orig}x{map_h_orig})", fontsize=11)
    (right_scatter,) = ax_right.plot([], [], "o", color="lime", markersize=8,
                                       markeredgecolor="white", markeredgewidth=1.5)

    # 网格（每隔 5m）
    for xm in range(0, int(field_w) + 1, 5):
        ax_right.axvline(x=xm * RIGHT_W / field_w, color="gray", alpha=0.3, lw=0.5)
    for ym in range(0, int(field_h) + 1, 5):
        ax_right.axhline(y=RIGHT_H - ym * RIGHT_H / field_h,
                         color="gray", alpha=0.3, lw=0.5)

    plt.tight_layout()

    # ── 6. 事件 ──
    def _update_title():
        n0 = min(len(image_points[0]), len(map_points[0]))
        n1 = min(len(image_points[1]), len(map_points[1]))
        ax_left.set_title(f"相机 ({cam_w}x{cam_h}) — {LAYER_NAMES[current_layer]}",
                          fontsize=11)
        fig.suptitle(
            f"左键左图 | 右键右图 | 当前:{LAYER_NAMES[current_layer]} | "
            f"地面:{n0}对 高地:{n1}对 | T:切换 U:撤销 S:保存 Q:退出",
            fontsize=9, family="monospace", y=0.99)

    def redraw():
        # 左图点 — 所有层都画
        lx_all, ly_all = [], []
        for layer in range(2):
            for px, py in image_points[layer]:
                lx_all.append(px / cam_sx)
                ly_all.append(py / cam_sy)
        left_scatter.set_data(lx_all, ly_all)

        # 右图点
        rx_all, ry_all = [], []
        for layer in range(2):
            for px, py in map_points[layer]:
                rx_all.append(px / map_sx)
                ry_all.append(py / map_sy)
        right_scatter.set_data(rx_all, ry_all)

        _update_title()
        fig.canvas.draw_idle()

    _update_title()

    def on_click(event):
        nonlocal current_layer
        if event.inaxes is None:
            return

        if event.inaxes == ax_left and event.button == MouseButton.LEFT:
            px = int(round(event.xdata * cam_sx))
            py = int(round(event.ydata * cam_sy))
            px = max(0, min(px, cam_w - 1))
            py = max(0, min(py, cam_h - 1))
            image_points[current_layer].append([px, py])
            n = len(image_points[current_layer])
            print(f"  [{LAYER_NAMES[current_layer]}] 图像点 P{n}: ({px}, {py})")
            redraw()

        elif event.inaxes == ax_right and event.button == MouseButton.RIGHT:
            mpx = int(round(event.xdata * map_sx))
            mpy = int(round(event.ydata * map_sy))
            mpx = max(0, min(mpx, map_w_orig - 1))
            mpy = max(0, min(mpy, map_h_orig - 1))
            map_points[current_layer].append([mpx, mpy])
            n = len(map_points[current_layer])
            # 日志中顺便显示世界坐标
            if is_portrait:
                wx = mpy / map_h_orig * field_w
                wy = (map_w_orig - mpx) / map_w_orig * field_h
            else:
                wx = mpx / map_w_orig * field_w
                wy = (map_h_orig - mpy) / map_h_orig * field_h
            print(f"  [{LAYER_NAMES[current_layer]}] 地图点 M{n}: "
                  f"像素({mpx},{mpy}) → 世界({wx:.2f},{wy:.2f})m")
            redraw()

    def on_key(event):
        nonlocal current_layer
        if event.key in ("t", "T"):
            current_layer = 1 - current_layer
            print(f"  切换到: {LAYER_NAMES[current_layer]}")
            redraw()
        elif event.key in ("u", "U"):
            layer = current_layer
            if len(map_points[layer]) > len(image_points[layer]):
                map_points[layer].pop()
            elif image_points[layer]:
                if len(map_points[layer]) == len(image_points[layer]):
                    map_points[layer].pop()
                image_points[layer].pop()
            n = len(image_points[layer])
            print(f"  撤销 → 地面:{min(len(image_points[0]),len(map_points[0]))}对 "
                  f"高地:{min(len(image_points[1]),len(map_points[1]))}对")
            redraw()
        elif event.key in ("s", "S"):
            _save()
        elif event.key in ("q", "Q", "escape"):
            plt.close()
            print("[退出]")

    def _save():
        """计算 Homography 并保存 — 完全参照 perspective_calibrator._on_save"""
        calib = {
            "calibration_time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "my_color": my_color,
            "map_image": os.path.basename(map_path),
            "map_w": map_w_orig,
            "map_h": map_h_orig,
            "map_is_portrait": is_portrait,
        }
        ok = False

        # 地面层
        n0 = min(len(image_points[0]), len(map_points[0]))
        if n0 >= 4:
            src = np.array(image_points[0][:n0], dtype=np.float64)
            dst = np.array(map_points[0][:n0], dtype=np.float64)
            H, status = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
            if H is not None:
                calib["H_ground"] = H.tolist()
                calib["H"] = H.tolist()
                calib["pixel_points_ground"] = image_points[0][:n0]
                calib["field_points_ground"] = map_points[0][:n0]
                inl = int(np.sum(status))
                print(f"  ✔ 地面层: {n0} 点, {inl} 内点, H=\n{H}")
                ok = True
            else:
                print("  ✘ 地面层 H 计算失败")
        elif n0 > 0:
            print(f"  ⚠ 地面层需 ≥4 对, 当前 {n0}")

        # 高地层
        n1 = min(len(image_points[1]), len(map_points[1]))
        if n1 >= 4:
            src = np.array(image_points[1][:n1], dtype=np.float64)
            dst = np.array(map_points[1][:n1], dtype=np.float64)
            H, status = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
            if H is not None:
                calib["H_highland"] = H.tolist()
                calib["pixel_points_highland"] = image_points[1][:n1]
                calib["field_points_highland"] = map_points[1][:n1]
                inl = int(np.sum(status))
                print(f"  ✔ 高地层: {n1} 点, {inl} 内点, H=\n{H}")
                ok = True
            else:
                print("  ✘ 高地层 H 计算失败")
        elif n1 > 0:
            print(f"  ⚠ 高地层需 ≥4 对, 当前 {n1}")

        if not ok:
            print("  ✘ 无有效标定结果")
            return

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, "w") as f:
            json.dump(calib, f, indent=2, ensure_ascii=False)
        print(f"  ✔ 已保存: {output_path}")

        # 重投影误差
        for layer_idx, name in enumerate(LAYER_NAMES):
            key = "H_ground" if layer_idx == 0 else "H_highland"
            if key not in calib:
                continue
            H = np.array(calib[key])
            n = min(len(image_points[layer_idx]), len(map_points[layer_idx]))
            src = np.array(image_points[layer_idx][:n], dtype=np.float64)
            dst = np.array(map_points[layer_idx][:n], dtype=np.float64)
            pred = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
            errs = np.linalg.norm(pred - dst, axis=1)
            print(f"  [{name}] 重投影误差(px): "
                  f"mean={np.mean(errs):.2f}, max={np.max(errs):.2f}, "
                  f"各点={[f'{e:.1f}' for e in errs]}")

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.show()


if __name__ == "__main__":
    main()
