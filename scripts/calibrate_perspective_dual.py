#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_perspective_dual.py — 双相机（左半场 / 右半场）透视变换矩阵标定

背景：lab 场地过大，一台相机看不全，用两台相机各负责一半，两个 detect 节点
      唯一的差别就是各自的单应矩阵 H。本脚本同一份代码跑两次，每次标一台：

    python3 scripts/calibrate_perspective_dual.py --cam left
    python3 scripts/calibrate_perspective_dual.py --cam right

默认输出（不覆盖 calibrate_perspective.py 的产物）：
    configs/perspective_calib_<scene>_left.json
    configs/perspective_calib_<scene>_right.json

与 calibrate_perspective.py 的差别：
  1. 只标平地（单层）：不切层、不算 H_highland、不读掩码；节点侧 H_highland
     缺失时本来就恒用地面 H，正好符合"只有平地"。
  2. 地图图可用 --map 指定；不给就用 lab_dual.yaml 里 map.image 那张。
     地图的世界范围依次找：--map-origin4 > <图名>_origin.txt > <图名>.yaml（map_server）。
  3. 可用 --map-crop 把地图里的一块矩形当成"输出坐标系"，让 H 直接落在节点
     小地图的像素坐标系里（节点 HomographyTransformer 只认像素坐标）。
  4. 输出 JSON 里额外记录了地图的物理范围（m/px、左上角世界坐标、物理尺寸），
     供节点换算场地坐标时使用（当前节点是写死 28x15 m，见脚本末尾提示）。
  5. 相机图：默认按 main_config.yaml 的 camera.mode 取；hik 模式直接走海康 MVS SDK
     （不用 HNURM 的 Camera/HKCam.py——那个封装写死 3072x2048 + BayerRG12，换型号
     就 sys.exit），分辨率/像素格式按相机实际值走，并把实际值和设备型号/SN 记进 JSON。

节点侧现状（本脚本不改节点）：detect.cpp 固定读 `configs/perspective_calib.json`
一个文件，所以两份标定结果怎么接给两个 detect 是节点侧的事，脚本只负责把两台
相机的 H 分别落到各自的 JSON 里。

操作：
  左键点【左图 = 相机画面】 → 相机特征点
  右键点【右图 = 地图】     → 同一个物理点在地图上的位置
  两组点按顺序配对：第 1 个相机点 ↔ 第 1 个地图点，以此类推
  U = 撤销上一点 | S = 计算 H 并保存 | Q / Esc = 退出 | --list-cameras = 列设备

选点建议：选该相机看得清、且在地图上位置明确的点，≥4 对（建议 6~8 对），
          尽量分散到画面四角、不要共线；两台相机各自选各自看得见的点即可，
          不需要两台都点同一个点。
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton

# 关掉 matplotlib 自带的快捷键：默认 s = 弹"保存图片"对话框、q = 直接关窗，
# 会和本脚本的 S(存 JSON) / Q(退出) 抢键（按 S 会弹出 "Error saving file"）。
for _k in ("keymap.save", "keymap.quit", "keymap.quit_all"):
    matplotlib.rcParams[_k] = []


def setup_cjk_font():
    """matplotlib 默认字体没有中日韩字形，中文标题会变成方框。有 Noto CJK 就用它。"""
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    for cand in ("Noto Sans CJK SC", "Noto Sans CJK JP", "Noto Sans CJK TC",
                 "Noto Sans Mono CJK SC", "WenQuanYi Micro Hei", "WenQuanYi Zen Hei",
                 "Source Han Sans SC", "Microsoft YaHei", "SimHei"):
        if cand in have:
            matplotlib.rcParams["font.family"] = "sans-serif"
            matplotlib.rcParams["font.sans-serif"] = [cand, "DejaVu Sans"]
            matplotlib.rcParams["axes.unicode_minus"] = False
            return cand
    return None


CJK_FONT = setup_cjk_font()

# ======================== 路径 ========================
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent                 # RM_radar_Cpp_2027
WORKSPACE_ROOT = PROJECT_ROOT.parent             # /home/syh/rm_lidar_2027

MAIN_CONFIG_PATH = PROJECT_ROOT / "configs" / "main_config.yaml"
# 海康 MVS 的 ctypes 封装（直接用 SDK，不经过 HNURM 的 Camera/HKCam.py：
# 那个封装写死 3072x2048 + BayerRG12，换型号会直接 sys.exit）
MVIMPORT_DIR = (WORKSPACE_ROOT / "HNURM-radar-2026" / "src" / "hnurm_radar"
                / "hnurm_radar" / "Camera" / "MvImport")

# 面板最大显示尺寸（按原图比例缩放，不拉伸）
LEFT_MAX = (860, 620)
RIGHT_MAX = (980, 720)
FIG_SIZE = (19.0, 8.6)

# 相机取帧稳定时间（秒）：打开相机后连续取帧这么久，取最后一帧当标定图
HIK_SETTLE_S = 1.2

# 节点侧 HomographyTransformer::mapToField 写死的场地尺寸（米）
NODE_FIELD_W = 28.0
NODE_FIELD_H = 15.0


# ======================== 基础工具 ========================
def load_config():
    try:
        from ruamel.yaml import YAML
        with open(MAIN_CONFIG_PATH, encoding="utf-8") as f:
            return YAML().load(f) or {}
    except Exception:
        import yaml
        with open(MAIN_CONFIG_PATH, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}


def resolve_path(p, must_exist=False):
    """相对路径：先按项目根解析，再按工作区根解析。"""
    if p is None:
        return None
    p = str(p)
    if os.path.isabs(p):
        cand = Path(p)
    else:
        cand = None
        for base in (PROJECT_ROOT, WORKSPACE_ROOT):
            if (base / p).exists():
                cand = base / p
                break
        if cand is None:
            cand = PROJECT_ROOT / p
    if must_exist and not cand.exists():
        raise FileNotFoundError(f"找不到文件: {p}  (试过: {cand})")
    return cand


def fit_display(iw, ih, max_w, max_h):
    """按比例缩到 max 以内，返回 (显示宽, 显示高)。"""
    s = min(max_w / float(iw), max_h / float(ih))
    return max(1, int(round(iw * s))), max(1, int(round(ih * s)))


def read_origin4(map_path):
    """读同目录下的 <stem>_origin.txt（make_2d_map.py 的产物）：
        x_lo y_lo x_hi y_hi —— 图像左下角与右上角的世界坐标。
    返回 (x_lo, y_lo, x_hi, y_hi) 或 None。"""
    cand = map_path.with_name(map_path.stem + "_origin.txt")
    if not cand.exists():
        return None
    try:
        parts = cand.read_text().split()
        if len(parts) < 4:
            return None
        v = [float(x) for x in parts[:4]]
        if v[2] <= v[0] or v[3] <= v[1]:
            return None
        return tuple(v)
    except Exception:
        return None


def read_map_server_yaml(map_path, map_w, map_h):
    """读同目录下的 <stem>.yaml（map_server / ros2 map 格式）：
        resolution: m/px, origin: [x, y, yaw] —— 图像**左下角**像素的世界坐标。
    返回 (x_lo, y_lo, x_hi, y_hi)（与 read_origin4 同口径：左下 + 右上）或 None。"""
    cand = map_path.with_name(map_path.stem + ".yaml")
    if not cand.exists():
        cand = map_path.with_name(map_path.stem + ".yml")
    if not cand.exists():
        return None
    try:
        import yaml
        d = yaml.safe_load(cand.read_text()) or {}
        res = float(d["resolution"])
        ox, oy = float(d["origin"][0]), float(d["origin"][1])
        yaw = float(d["origin"][2]) if len(d.get("origin", [])) > 2 else 0.0
        if res <= 0:
            return None
        if abs(yaw) > 1e-6:
            print(f"[警告] {cand.name} 的 origin 带 yaw={yaw}（地图相对世界有旋转），"
                  f"本工具与节点都按轴对齐处理，旋转会被忽略")
        return (ox, oy, ox + map_w * res, oy + map_h * res)
    except Exception as e:
        print(f"[警告] 读 {cand.name} 失败: {e}")
        return None


def default_map_path():
    """--map 没给时：用 lab_dual 配置里的那张图（唯一出处）。"""
    cfg = WORKSPACE_ROOT / "lab_dual_ws" / "src" / "lab_dual" / "config" / "lab_dual.yaml"
    try:
        import yaml
        d = yaml.safe_load(cfg.read_text()) or {}
        img = (d.get("map") or {}).get("image")
        if img:
            print(f"[地图] 取自 {cfg.name}: {img}")
            return img
    except Exception:
        pass
    return "map_2d.png"


# ======================== 相机 ========================
def _mvs():
    """导入海康 MVS 的 ctypes 封装。"""
    os.environ.setdefault("MVCAM_COMMON_RUNENV", "/opt/MVS/lib")
    if str(MVIMPORT_DIR) not in sys.path:
        sys.path.insert(0, str(MVIMPORT_DIR))
    import MvCameraControl_class as mv
    return mv


def _pixfmt_table(mv):
    return {getattr(mv, n): n.replace("PixelType_Gvsp_", "")
            for n in dir(mv) if n.startswith("PixelType_Gvsp_")}


def list_hik_devices():
    """枚举海康设备，返回 [(index, model, serial, kind)]。"""
    mv = _mvs()
    from ctypes import cast, POINTER

    dev_list = mv.MV_CC_DEVICE_INFO_LIST()
    ret = mv.MvCamera.MV_CC_EnumDevices(mv.MV_GIGE_DEVICE | mv.MV_USB_DEVICE, dev_list)
    if ret != 0:
        raise RuntimeError(f"MV_CC_EnumDevices 失败, ret=0x{ret:x}")
    out = []
    for i in range(dev_list.nDeviceNum):
        info = cast(dev_list.pDeviceInfo[i], POINTER(mv.MV_CC_DEVICE_INFO)).contents
        if info.nTLayerType == mv.MV_GIGE_DEVICE:
            g = info.SpecialInfo.stGigEInfo
            model = bytes(g.chModelName).split(b"\x00")[0].decode(errors="replace")
            serial = bytes(g.chSerialNumber).split(b"\x00")[0].decode(errors="replace")
            out.append((i, model, serial, "GigE"))
        elif info.nTLayerType == mv.MV_USB_DEVICE:
            u = info.SpecialInfo.stUsb3VInfo
            model = bytes(u.chModelName).split(b"\x00")[0].decode(errors="replace")
            serial = bytes(u.chSerialNumber).split(b"\x00")[0].decode(errors="replace")
            out.append((i, model, serial, "USB"))
        else:
            out.append((i, "?", "?", f"layer={info.nTLayerType}"))
    return out


def _frame_to_bgr(mv, cam, frame, names):
    """按该帧实际的像素格式转成 BGR。返回 (BGR 图 或 None, 像素格式名)。"""
    from ctypes import POINTER, c_ubyte, byref, memset, sizeof

    fi = frame.stFrameInfo
    w, h, pix = int(fi.nWidth), int(fi.nHeight), int(fi.enPixelType)
    pname = names.get(pix, f"0x{pix:x}")
    npix = w * h

    if pname == "BGR8_Packed":
        v = np.ctypeslib.as_array(frame.pBufAddr, shape=(npix * 3,)).reshape(h, w, 3)
        return v.copy(), pname
    if pname == "RGB8_Packed":
        v = np.ctypeslib.as_array(frame.pBufAddr, shape=(npix * 3,)).reshape(h, w, 3)
        return cv2.cvtColor(v, cv2.COLOR_RGB2BGR), pname
    if pname == "Mono8":
        v = np.ctypeslib.as_array(frame.pBufAddr, shape=(npix,)).reshape(h, w)
        return cv2.cvtColor(v, cv2.COLOR_GRAY2BGR), pname

    # Bayer / Mono10 / Mono12 等交给 SDK 转 BGR8
    dst = np.empty((h, w, 3), np.uint8)
    param = mv.MV_CC_PIXEL_CONVERT_PARAM_EX()
    memset(byref(param), 0, sizeof(param))
    param.nWidth = w
    param.nHeight = h
    param.enSrcPixelType = pix
    param.pSrcData = frame.pBufAddr
    param.nSrcDataLen = int(fi.nFrameLen)
    param.enDstPixelType = mv.PixelType_Gvsp_BGR8_Packed
    param.pDstBuffer = dst.ctypes.data_as(POINTER(c_ubyte))
    param.nDstBufferSize = int(dst.size)
    if cam.MV_CC_ConvertPixelTypeEx(param) != 0:
        return None, pname
    return dst, pname


def grab_hik_frame(index, want=(3072, 2048), settle=HIK_SETTLE_S):
    """打开第 index 台相机取一帧（BGR）。

    分辨率/像素格式尽量对齐 hik_camera_node（3072x2048 + BayerRG8），设不上就用相机
    当前值——和节点那边"设不上只 warn、之后按实际值走"的行为一致，这样标定图的像素
    坐标系才和节点实际喂给 detect 的图一致。"""
    mv = _mvs()
    from ctypes import cast, POINTER
    names = _pixfmt_table(mv)

    dev_list = mv.MV_CC_DEVICE_INFO_LIST()
    ret = mv.MvCamera.MV_CC_EnumDevices(mv.MV_GIGE_DEVICE | mv.MV_USB_DEVICE, dev_list)
    if ret != 0:
        raise RuntimeError(f"MV_CC_EnumDevices 失败, ret=0x{ret:x}")
    if not (0 <= index < dev_list.nDeviceNum):
        raise RuntimeError(f"设备序号 {index} 超出范围 (0~{dev_list.nDeviceNum - 1})")
    info = cast(dev_list.pDeviceInfo[index], POINTER(mv.MV_CC_DEVICE_INFO)).contents

    cam = mv.MvCamera()
    ret = cam.MV_CC_CreateHandle(info)
    if ret != 0:
        raise RuntimeError(f"MV_CC_CreateHandle 失败, ret=0x{ret:x}")
    ret = cam.MV_CC_OpenDevice(mv.MV_ACCESS_Exclusive, 0)
    if ret != 0:
        cam.MV_CC_DestroyHandle()
        raise RuntimeError(
            f"打开设备失败 ret=0x{ret:x}：多半是这台相机正被 hik_camera_node / detect 占着"
            f"（MVS 是独占访问），先把那些节点停掉再标定")

    img = pname = None
    cur_wh = None
    try:
        cam.MV_CC_SetEnumValue("TriggerMode", mv.MV_TRIGGER_MODE_OFF)
        for key, val in (("Width", want[0]), ("Height", want[1])):
            if cam.MV_CC_SetIntValue(key, val) != 0:
                print(f"[相机] {key}={val} 设不上（该型号不支持这个分辨率），按相机当前值走")
        if cam.MV_CC_SetEnumValue("PixelFormat", mv.PixelType_Gvsp_BayerRG8) != 0:
            print("[相机] BayerRG8 设不上，按相机当前像素格式走")

        def get_int(key):
            v = mv.MVCC_INTVALUE()
            cam.MV_CC_GetIntValue(key, v)
            return int(v.nCurValue)

        cur_wh = (get_int("Width"), get_int("Height"))
        cam.MV_CC_StartGrabbing()
        t0 = time.time()
        while time.time() - t0 < settle:
            frame = mv.MV_FRAME_OUT()
            if cam.MV_CC_GetImageBuffer(frame, 1000) != 0:
                continue
            try:
                f, name = _frame_to_bgr(mv, cam, frame, names)
            finally:
                cam.MV_CC_FreeImageBuffer(frame)
            if f is not None:
                img, pname = f, name
        cam.MV_CC_StopGrabbing()
    finally:
        cam.MV_CC_CloseDevice()
        cam.MV_CC_DestroyHandle()

    if img is None:
        raise RuntimeError("相机在线但一帧都没出（看看曝光/触发模式/像素格式）")
    print(f"[相机] 实际出图: {img.shape[1]}x{img.shape[0]}, 像素格式 {pname}")
    return img, {"device_index": index, "width": int(img.shape[1]), "height": int(img.shape[0]),
                 "pixel_format": pname, "resolution_after_open": list(cur_wh)}


def grab_video_frame(source):
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开视频源: {source}")
    img = None
    t0 = time.time()
    while time.time() - t0 < 3.0:
        ret, f = cap.read()
        if ret:
            img = f
            break
        time.sleep(0.05)
    cap.release()
    if img is None:
        raise RuntimeError(f"视频源无帧: {source}")
    return img


def capture_camera_frame(args, cfg):
    """按优先级取标定用相机图：--image > main_config.camera.mode。
    返回 (BGR 图, 来源说明 dict)。"""
    if args.image:
        img = cv2.imread(str(resolve_path(args.image, must_exist=True)))
        if img is None:
            raise RuntimeError(f"无法读取图片: {args.image}")
        return img, {"source": "image", "path": str(resolve_path(args.image))}

    cam_cfg = cfg.get("camera", {}) or {}
    mode = args.mode or cam_cfg.get("mode", "test")

    if mode == "hik":
        devices = list_hik_devices()
        if not devices:
            raise RuntimeError("没有枚举到海康相机")
        print("[相机] 枚举到的海康设备：")
        for idx, model, serial, kind in devices:
            print(f"        [{idx}] {model}  SN={serial}  ({kind})")

        index = None
        serial = None
        if args.serial:
            for idx, model, sn, kind in devices:
                if sn == args.serial:
                    index, serial = idx, sn
                    break
            if index is None:
                raise RuntimeError(f"没有序列号为 {args.serial} 的设备")
        else:
            index = args.cam_index if args.cam_index is not None else (0 if args.cam == "left" else 1)
            if index >= len(devices):
                raise RuntimeError(f"--cam-index {index} 超出范围 (0~{len(devices) - 1})")
            serial = devices[index][2]
            if len(devices) > 1:
                print("[相机] 提示：两台以上设备时序号不保证稳定，建议用 --serial 固定")

        model = devices[index][1]
        print(f"[相机] 使用设备 [{index}] {model} SN={serial}，取帧中…")
        img, gmeta = grab_hik_frame(index)
        info = {"source": "hik", "device_index": index, "serial": serial, "model": model}
        info.update(gmeta)
        return img, info

    if mode == "video":
        source = cam_cfg.get("video_source", 0)
        if isinstance(source, str):
            source = str(resolve_path(source))
        img = grab_video_frame(source)
        print(f"[相机] 视频源取到一帧: {img.shape[1]}x{img.shape[0]}")
        return img, {"source": "video", "path": str(source)}

    # test 模式：静态图
    test_img = cam_cfg.get("test_image", "")
    if not test_img:
        raise RuntimeError("camera.mode=test 但没有 camera.test_image 配置，请用 --image 指定图片")
    path = resolve_path(test_img, must_exist=True)
    img = cv2.imread(str(path))
    if img is None:
        raise RuntimeError(f"无法读取图片: {path}")
    print(f"[相机] 使用测试图片: {path}  ({img.shape[1]}x{img.shape[0]})")
    return img, {"source": "test_image", "path": str(path)}


# ======================== 标定会话 ========================
class DualCalib:
    """左图=相机画面，右图=地图；左键打相机点，右键打地图点；只算平地 H。"""

    def __init__(self, cam_img, map_img, meta):
        self.cam_img = cam_img
        self.map_img = map_img
        self.meta = meta                     # 见 build_meta()

        self.cam_h, self.cam_w = cam_img.shape[:2]
        self.map_h, self.map_w = map_img.shape[:2]

        self.cam_pts = []                    # 相机像素 [[px,py], ...]
        self.map_pts = []                    # 地图像素（点击图坐标系）[[px,py], ...]
        self.saved_once = False
        self.dirty = False          # 有未保存的点
        self.quit_armed = False     # Q 按过一次（防手滑丢掉没保存的点）

        # 显示缩放
        self.cam_dw, self.cam_dh = fit_display(self.cam_w, self.cam_h, *LEFT_MAX)
        self.map_dw, self.map_dh = fit_display(self.map_w, self.map_h, *RIGHT_MAX)
        self.cam_sx, self.cam_sy = self.cam_w / self.cam_dw, self.cam_h / self.cam_dh
        self.map_sx, self.map_sy = self.map_w / self.map_dw, self.map_h / self.map_dh

        self.fig = None
        self.ax_l = self.ax_r = None
        self.scat_l = self.scat_r = None
        self.texts = []

    # ---------- 建界面 ----------
    def build(self):
        meta = self.meta
        fig, (ax_l, ax_r) = plt.subplots(1, 2, figsize=FIG_SIZE)
        self.fig, self.ax_l, self.ax_r = fig, ax_l, ax_r
        try:
            fig.canvas.manager.set_window_title(
                f"Perspective Calibration [{meta['cam']}] -> {Path(meta['output']).name}")
        except Exception:
            pass

        ax_l.imshow(cv2.cvtColor(cv2.resize(self.cam_img, (self.cam_dw, self.cam_dh)),
                                 cv2.COLOR_BGR2RGB))
        ax_l.set_title(f"相机 [{meta['cam']}]  {self.cam_w}x{self.cam_h}"
                       f"  ({meta['cam_source']})", fontsize=11)
        (self.scat_l,) = ax_l.plot([], [], "o", color="cyan", markersize=9,
                                   markeredgecolor="white", markeredgewidth=1.5)

        ax_r.imshow(cv2.cvtColor(cv2.resize(self.map_img, (self.map_dw, self.map_dh)),
                                 cv2.COLOR_BGR2RGB))
        title_r = f"地图 {self.map_w}x{self.map_h}  {Path(meta['map_path']).name}"
        if meta.get("extent_wh"):
            title_r += f"  ({meta['extent_wh'][0]:.2f}x{meta['extent_wh'][1]:.2f} m"
            title_r += f", {meta['res'][0] * 100:.1f} cm/px)"
        ax_r.set_title(title_r, fontsize=11)
        (self.scat_r,) = ax_r.plot([], [], "o", color="lime", markersize=9,
                                   markeredgecolor="white", markeredgewidth=1.5)

        self._draw_grid()
        plt.tight_layout(rect=[0, 0, 1, 0.93])   # 顶部留出 suptitle 的位置

        fig.canvas.mpl_connect("button_press_event", self.on_click)
        fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.redraw()

    def _draw_grid(self):
        """按地图物理范围画 1 m / 5 m 网格；范围未知就退化成按 28x15 假定。"""
        meta = self.meta
        if meta.get("extent_wh"):
            x_lo, y_hi = meta["origin_xy"]
            res_x, res_y = meta["res"]
            w_m, h_m = meta["extent_wh"]
            x0, y0 = x_lo, y_hi - h_m
            note = ""
        else:
            x0, y0 = 0.0, 0.0
            res_x = res_y = None
            w_m, h_m = NODE_FIELD_W, NODE_FIELD_H
            note = "（物理范围未知，网格按 28x15 m 假定）"
        self.ax_r.set_title(self.ax_r.get_title() + note, fontsize=10)

        step = 5 if max(w_m, h_m) > 20 else 1
        xv = x0
        while xv <= x0 + w_m + 1e-9:
            d = (xv - x0) / w_m * self.map_dw
            self.ax_r.axvline(x=d, color="gray", alpha=0.30, lw=0.5)
            if abs(xv - round(xv)) < 1e-6:
                self.ax_r.text(d, 6, f"{int(round(xv))}", color="yellow", fontsize=7)
            xv += step
        yv = y0
        while yv <= y0 + h_m + 1e-9:
            d = self.map_dh - (yv - y0) / h_m * self.map_dh
            self.ax_r.axhline(y=d, color="gray", alpha=0.30, lw=0.5)
            if abs(yv - round(yv)) < 1e-6:
                self.ax_r.text(4, d - 4, f"{int(round(yv))}", color="yellow", fontsize=7)
            yv += step

    # ---------- 重绘 ----------
    def redraw(self):
        lx = [p[0] / self.cam_sx for p in self.cam_pts]
        ly = [p[1] / self.cam_sy for p in self.cam_pts]
        self.scat_l.set_data(lx, ly)

        rx = [p[0] / self.map_sx for p in self.map_pts]
        ry = [p[1] / self.map_sy for p in self.map_pts]
        self.scat_r.set_data(rx, ry)

        for t in self.texts:
            t.remove()
        self.texts = []
        for i, (x, y) in enumerate(zip(lx, ly)):
            self.texts.append(self.ax_l.text(x + 6, y - 6, str(i + 1), color="cyan", fontsize=9))
        for i, (x, y) in enumerate(zip(rx, ry)):
            self.texts.append(self.ax_r.text(x + 6, y - 6, str(i + 1), color="lime", fontsize=9))

        n = min(len(self.cam_pts), len(self.map_pts))
        self.fig.suptitle(
            f"左键点相机 / 右键点地图 (按序号配对) | 已配对 {n} 对 "
            f"(相机 {len(self.cam_pts)} / 地图 {len(self.map_pts)}) | "
            f"U:撤销  S:保存  Q:退出 | 输出 {self.meta['output']}",
            fontsize=10, y=0.985)
        self.fig.canvas.draw_idle()

    # ---------- 事件 ----------
    def on_click(self, event):
        if event.inaxes is None:
            return
        if event.inaxes is self.ax_l and event.button == MouseButton.LEFT:
            px = int(round((event.xdata + 0.5) * self.cam_sx - 0.5))
            py = int(round((event.ydata + 0.5) * self.cam_sy - 0.5))
            px = max(0, min(px, self.cam_w - 1))
            py = max(0, min(py, self.cam_h - 1))
            self.cam_pts.append([px, py])
            self.dirty = True
            self.quit_armed = False
            print(f"  相机点 P{len(self.cam_pts)}: ({px}, {py})")
            self.redraw()
        elif event.inaxes is self.ax_r and event.button == MouseButton.RIGHT:
            mx = int(round((event.xdata + 0.5) * self.map_sx - 0.5))
            my = int(round((event.ydata + 0.5) * self.map_sy - 0.5))
            mx = max(0, min(mx, self.map_w - 1))
            my = max(0, min(my, self.map_h - 1))
            self.map_pts.append([mx, my])
            self.dirty = True
            self.quit_armed = False
            msg = f"  地图点 M{len(self.map_pts)}: 像素({mx}, {my})"
            if self.meta.get("extent_wh"):
                x_lo, y_hi = self.meta["origin_xy"]
                res_x, res_y = self.meta["res"]
                wx = x_lo + mx * res_x
                wy = y_hi - my * res_y
                msg += f" → 世界({wx:.2f}, {wy:.2f}) m"
            print(msg)
            self.redraw()

    def on_key(self, event):
        k = (event.key or "").lower()
        if k == "u":
            self.undo()
        elif k == "s":
            self.save()
        elif k in ("q", "escape"):
            n = min(len(self.cam_pts), len(self.map_pts))
            if self.dirty and n > 0 and not self.quit_armed:
                self.quit_armed = True
                print(f"  ⚠ 还有 {n} 对点没保存（{self.meta['output']}）："
                      f"按 S 保存，再按一次 Q 直接退出")
                return
            plt.close(self.fig)
            print("[退出]")
        elif k == "t":
            print("  本工具只标平地，没有高地层（节点在无 H_highland 时恒用地面 H）")

    def undo(self):
        if len(self.map_pts) > len(self.cam_pts):
            self.map_pts.pop()
        elif self.cam_pts:
            self.cam_pts.pop()
            if len(self.map_pts) > len(self.cam_pts):
                self.map_pts.pop()
        print(f"  撤销 → 已配对 {min(len(self.cam_pts), len(self.map_pts))} 对")
        self.dirty = True
        self.redraw()

    # ---------- 保存 ----------
    def save(self):
        out = Path(self.meta["output"])
        if out.exists() and not self.saved_once and not self.meta["force"]:
            print(f"  ✘ 目标已存在: {out}\n"
                  f"    没有覆盖。加 --force 覆盖，或用 --output 换个名字。")
            return False

        n = min(len(self.cam_pts), len(self.map_pts))
        if n < 4:
            print(f"  ⚠ 至少需要 4 对点，当前 {n} 对")
            return False
        if n == 4:
            print("  ⚠ 只有 4 对点：H 被唯一确定，重投影误差必然为 0，看不出选点好坏；"
                  "建议 6~8 对")

        src = np.array(self.cam_pts[:n], dtype=np.float64)
        dst = np.array(self.map_pts[:n], dtype=np.float64)
        H, status = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        if H is None:
            print("  ✘ findHomography 失败（点可能共线或重复）")
            return False
        inl = int(np.sum(status)) if status is not None else -1
        print(f"  ✔ {n} 对点, {inl} 内点, H(点击图坐标系)=\n{np.array2string(H, precision=6)}")

        # 落到输出坐标系（可选裁剪/缩放平移）
        H_out, dst_out = apply_output_frame(H, dst, self.meta)
        out_w, out_h = self.meta["out_size"]

        pred = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H_out).reshape(-1, 2)
        errs = np.linalg.norm(pred - dst_out, axis=1)
        print(f"  [重投影误差] mean={errs.mean():.2f}px  max={errs.max():.2f}px  "
              f"各点={[round(float(e), 1) for e in errs]}")
        if errs.max() > 20:
            print("  ⚠ 有点误差 >20px：检查是不是某个点配错了序号")

        payload = self.make_payload(H_out, errs, n)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        self.saved_once = True
        self.dirty = False
        print(f"  ✔ 已保存: {out}")

        report_node_compat(self.meta, H_out, self.cam_w, self.cam_h)
        return True

    def make_payload(self, H_out, errs, n):
        meta = self.meta
        out_w, out_h = meta["out_size"]
        payload = {
            "calibration_time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "side": meta["cam"],
            "scene": meta["scene"],
            "my_color": meta["my_color"],
            "camera": meta["cam_info"],
            "map_image": Path(meta["map_path"]).name,
            "map_path": str(meta["map_path"]),
            "map_w": int(out_w),
            "map_h": int(out_h),
            "map_is_portrait": bool(out_h > out_w),
            "map_crop_xywh": list(meta["crop"]) if meta.get("crop") else None,
            "map_res_m_per_px": list(meta["res"]) if meta.get("res") else None,
            "map_origin_xy_m": list(meta["origin_xy"]) if meta.get("origin_xy") else None,
            "map_extent_m": list(meta["extent_wh"]) if meta.get("extent_wh") else None,
            "H_ground": H_out.tolist(),
            "H": H_out.tolist(),
            "pixel_points_ground": self.cam_pts[:n],
            "field_points_ground": [list(map(float, p)) for p in
                                    np.array(self.map_pts[:n], dtype=np.float64)],
            "reprojection_error_px": {
                "mean": float(errs.mean()),
                "max": float(errs.max()),
                "per_point": [float(e) for e in errs],
            },
        }
        return payload


# ======================== 坐标系换算 ========================
def apply_output_frame(H_click, dst_click, meta):
    """把"点击图坐标系"的 H（及目标点）换到"输出坐标系"。
    输出坐标系 = 点击图整幅（无 --map-crop），或点击图里的 crop 矩形（有 --map-crop）。"""
    crop = meta.get("crop")
    if not crop:
        return H_click, np.asarray(dst_click, dtype=np.float64)
    x, y, w, h = crop
    T = np.array([[1.0, 0.0, -float(x)],
                  [0.0, 1.0, -float(y)],
                  [0.0, 0.0, 1.0]])
    H_out = T @ H_click
    H_out = H_out / H_out[2, 2]
    return H_out, np.asarray(dst_click, dtype=np.float64) - np.array([[x, y]], dtype=np.float64)


def build_meta(args, cfg, cam_info):
    scene = args.scene or (cfg.get("global", {}) or {}).get("scene", "lab")
    my_color = (cfg.get("global", {}) or {}).get("my_color", "Blue")

    map_path = resolve_path(args.map or default_map_path(), must_exist=True)
    map_img = cv2.imread(str(map_path), cv2.IMREAD_COLOR)
    if map_img is None:
        raise RuntimeError(f"无法读取地图: {map_path}")
    map_h, map_w = map_img.shape[:2]

    origin4 = None
    origin_src = "无"
    if args.map_origin4:
        origin4 = tuple(float(v) for v in args.map_origin4)
        origin_src = "--map-origin4"
    else:
        origin4 = read_origin4(map_path)
        if origin4:
            origin_src = f"{map_path.stem}_origin.txt"
        else:
            origin4 = read_map_server_yaml(map_path, map_w, map_h)
            if origin4:
                origin_src = f"{map_path.stem}.yaml (map_server)"
    print(f"[地图] {map_path.name} {map_w}x{map_h}, 世界范围来源: {origin_src}")

    res = None
    if origin4:
        x_lo, y_lo, x_hi, y_hi = origin4
        res = ((x_hi - x_lo) / map_w, (y_hi - y_lo) / map_h)
        if abs(res[0] - res[1]) > 0.05 * res[0]:
            print(f"[警告] {map_path.name} 的 x/y 分辨率不一致: "
                  f"{res[0]:.5f} vs {res[1]:.5f} m/px，网格只作参考")

    crop = tuple(int(v) for v in args.map_crop) if args.map_crop else None
    if crop:
        x, y, w, h = crop
        if x < 0 or y < 0 or w <= 0 or h <= 0 or x + w > map_w or y + h > map_h:
            raise RuntimeError(f"--map-crop {crop} 超出地图范围 ({map_w}x{map_h})")
        out_size = (w, h)
    else:
        out_size = (map_w, map_h)

    meta = {
        "cam": args.cam,
        "scene": scene,
        "my_color": my_color,
        "map_path": map_path,
        "map_size": (map_w, map_h),
        "crop": crop,
        "out_size": out_size,
        "res": res,
        "origin_xy": (origin4[0], origin4[3]) if origin4 else None,   # 地图左上角世界坐标
        "extent_wh": (map_w * res[0], map_h * res[1]) if res else None,
        "cam_info": cam_info,
        "cam_source": cam_info.get("source", "?"),
        "force": args.force,
        "output": str(resolve_path(args.output) if args.output else
                      PROJECT_ROOT / "configs" / f"perspective_calib_{scene}_{args.cam}.json"),
    }
    if crop and res:
        # 输出帧左上角的世界坐标
        meta["origin_xy"] = (origin4[0] + crop[0] * res[0], origin4[3] - crop[1] * res[1])
        meta["extent_wh"] = (crop[2] * res[0], crop[3] * res[1])
    return map_img, meta


def report_node_compat(meta, H_out, cam_w, cam_h):
    """打印"节点能不能直接用这份 H"的判断 + 相机覆盖范围。"""
    ext = meta.get("extent_wh")
    print("  ---- 节点兼容性 ----")
    if ext:
        ok = (abs(ext[0] - NODE_FIELD_W) / NODE_FIELD_W < 0.005 and
              abs(ext[1] - NODE_FIELD_H) / NODE_FIELD_H < 0.005)
        print(f"  输出坐标系物理尺寸: {ext[0]:.2f} x {ext[1]:.2f} m"
              f"  (地图 {meta['out_size'][0]}x{meta['out_size'][1]} px, "
              f"{meta['res'][0] * 100:.1f} cm/px)")
        if ok:
            print("  ✔ 与主工程节点写死的 28x15 m 一致，主程序 detect 也能直接用")
        else:
            print("  ⚠ 主工程 detect 的 HomographyTransformer::mapToField 把整幅地图固定当 28x15 m 换算，"
                  "与上面的实际尺寸不符\n"
                  "    → 主程序那套：要么改用落在 28x15 m 上的坐标系（--map-crop 或标 2800x1500 的图），\n"
                  "      要么让节点读本 JSON 的 map_res_m_per_px / map_origin_xy_m / map_extent_m\n"
                  "    → lab_dual 那套（camera_node/detect_node/display_node）：本来就读本 JSON 的"
                  " map_res_m_per_px，不受影响")
    else:
        print("  地图物理范围未知（没找到 <图名>_origin.txt / <图名>.yaml，也没给 --map-origin4），"
              "无法判断换算是否正确")

    # 相机画面四角落到输出地图上的位置
    corners = np.array([[0, 0], [cam_w - 1, 0], [cam_w - 1, cam_h - 1], [0, cam_h - 1]],
                       dtype=np.float64)
    p = cv2.perspectiveTransform(corners.reshape(-1, 1, 2), H_out).reshape(-1, 2)
    inside = int(np.sum((p[:, 0] >= 0) & (p[:, 0] < meta["out_size"][0]) &
                        (p[:, 1] >= 0) & (p[:, 1] < meta["out_size"][1])))
    print("  相机画面四角 → 输出地图像素: " +
          ", ".join(f"({q[0]:.0f},{q[1]:.0f})" for q in p) + f"   （{inside}/4 在输出图内）")
    if meta.get("res") and meta.get("origin_xy"):
        x_lo, y_hi = meta["origin_xy"]
        wx = x_lo + p[:, 0] * meta["res"][0]
        wy = y_hi - p[:, 1] * meta["res"][1]
        print(f"  → 世界坐标覆盖: X [{wx.min():.2f}, {wx.max():.2f}]  "
              f"Y [{wy.min():.2f}, {wy.max():.2f}] m")


# ======================== 入口 ========================
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="双相机（左半/右半）透视变换矩阵标定 — 输出 perspective_calib_<scene>_<cam>.json",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "例：\n"
            "  python3 scripts/calibrate_perspective_dual.py --cam left\n"
            "  python3 scripts/calibrate_perspective_dual.py --cam right --cam-index 1\n"
            "  python3 scripts/calibrate_perspective_dual.py --list-cameras\n"
            "  # 用已存好的相机截图标（相机被节点占用时）\n"
            "  python3 scripts/calibrate_perspective_dual.py --cam left --image /tmp/left.jpg\n"
            "  # 点大地图 map_2d.png，但把 H 落到 28x15 m 那一块（节点可直接用）\n"
            "  python3 scripts/calibrate_perspective_dual.py --cam left --map-crop X Y 2800 1500\n"
        ))
    p.add_argument("--cam", choices=["left", "right"], default="left",
                   help="标哪台相机（决定默认输出文件名，默认 left）")
    p.add_argument("--scene", default=None, help="场景名，默认取 main_config.yaml 的 global.scene")
    p.add_argument("--map", default=None,
                   help="点击用的地图图；不给就用 lab_dual.yaml 里 map.image 那张")
    p.add_argument("--map-crop", nargs=4, type=int, metavar=("X", "Y", "W", "H"), default=None,
                   help="把地图里这块矩形当作输出坐标系（H 的输出像素系），"
                        "例如地图整幅是 3115x2265 时给 '2800 1500' 那一块的偏移")
    p.add_argument("--map-origin4", nargs=4, type=float, metavar=("X_LO", "Y_LO", "X_HI", "Y_HI"),
                   default=None,
                   help="地图左下/右上角的世界坐标（同 make_2d_map.py 的 <stem>_origin.txt）；"
                        "不给就自动读同目录同名 _origin.txt")
    p.add_argument("--image", default=None, help="直接指定相机图（跳过采集）")
    p.add_argument("--mode", default=None, choices=["hik", "video", "test"],
                   help="覆盖 main_config.yaml 的 camera.mode")
    p.add_argument("--cam-index", type=int, default=None,
                   help="海康设备序号（左=0/右=1 是默认值，实际顺序见 --list-cameras）")
    p.add_argument("--serial", default=None, help="按海康相机序列号选设备（两台同型号时更稳）")
    p.add_argument("--output", default=None, help="输出 JSON 路径（默认 configs/perspective_calib_<scene>_<cam>.json）")
    p.add_argument("--force", action="store_true", help="允许覆盖已存在的输出文件")
    p.add_argument("--list-cameras", action="store_true", help="只列出海康设备再退出")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    if args.list_cameras:
        for idx, model, serial, kind in list_hik_devices():
            print(f"[{idx}] {model}  SN={serial}  ({kind})")
        return 0

    cfg = load_config()
    cam_img, cam_info = capture_camera_frame(args, cfg)
    map_img, meta = build_meta(args, cfg, cam_info)

    print("=" * 68)
    print("  双相机透视标定 — 只标平地")
    print("=" * 68)
    print(f"  相机 [{args.cam}]: {cam_img.shape[1]}x{cam_img.shape[0]}  ({cam_info['source']})")
    print(f"  地图: {meta['map_path']}  {meta['map_size'][0]}x{meta['map_size'][1]}")
    if meta.get("extent_wh"):
        print(f"        物理范围 {meta['extent_wh'][0]:.2f} x {meta['extent_wh'][1]:.2f} m, "
              f"{meta['res'][0] * 100:.1f} cm/px")
    if meta.get("crop"):
        print(f"        输出坐标系 = 图上矩形 {meta['crop']} → {meta['out_size'][0]}x{meta['out_size'][1]} px")
    print(f"  输出: {meta['output']}")
    print("=" * 68)
    print("  左键点左图(相机) | 右键点右图(地图) | U:撤销 S:保存 Q:退出")
    if CJK_FONT is None:
        print("  提示: 没找到中文字体，标题里的中文会显示成方框（不影响功能）")
    print()

    calib = DualCalib(cam_img, map_img, meta)
    calib.build()
    plt.show()
    print(f"  下次标另一台: python3 scripts/calibrate_perspective_dual.py "
          f"--cam {'right' if args.cam == 'left' else 'left'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
