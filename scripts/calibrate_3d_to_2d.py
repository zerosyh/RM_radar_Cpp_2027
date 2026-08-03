#!/usr/bin/env python3
"""
3D PCD → 2D Map 手动校准工具

功能：
  - 加载 2D 地图 (PNG) 作为背景，坐标原点在左下角
  - 加载 3D PCD 点云，以俯视图 (X-Y) 散点叠加显示
  - 键盘手动调整平移/旋转，对齐两套坐标系
  - 输出变换参数，用于后续对 PCD 做预处理

操作说明：
  W/A/S/D      平移 PCD（上/左/下/右）          步长 0.2m
  Shift+W/A/S/D 微调平移                          步长 0.05m
  Q/E          旋转 PCD（逆时针/顺时针）          步长 1°
  Shift+Q/E    微调旋转                            步长 0.1°
  Z/X          按 PCD 的 Z 高度过滤（显示不同层）
  1/2          调整点云透明度
  R            重置所有变换
  Enter        打印当前变换矩阵并保存
  H            显示帮助
  Esc          退出

输出：
  - 终端打印 (dx, dy, theta_deg) 及 4x4 变换矩阵
  - 自动保存变换后的 PCD 到 source/pointclouds/background/background2_aligned.pcd
"""

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backend_bases import KeyEvent
import matplotlib.image as mpimg
import struct
import os
import sys
import re
from pathlib import Path

# ======================== 配置 ========================
PROJECT_ROOT = Path(__file__).resolve().parent.parent

SCENE = "lab"  # "lab" or "competition"

def load_config():
    try:
        from ruamel.yaml import YAML
        yaml = YAML()
        config_path = PROJECT_ROOT / "configs" / "main_config.yaml"
        with open(config_path) as f:
            return yaml.load(f) or {}
    except Exception:
        return {}

cfg = load_config()
scene_cfg = cfg.get("scenes", {}).get(SCENE, {})

MAP_2D_PATH  = str(PROJECT_ROOT / scene_cfg.get("std_map", "source/maps/lab/map_2d_cropped.png"))
PCD_PATH     = str(PROJECT_ROOT / scene_cfg.get("pcd_file", "source/pointclouds/background/background2.pcd"))
FIELD_W      = float(scene_cfg.get("field_width", 28.0))
FIELD_H      = float(scene_cfg.get("field_height", 15.0))
OUTPUT_PCD   = str(PROJECT_ROOT / "source/pointclouds/background/background2_aligned.pcd")

# 显示参数
DISPLAY_MAX_PTS = 30000  # 渲染最多采样点数


# ======================== PCD 读取 ========================
def read_pcd_binary(path):
    with open(path, "rb") as f:
        raw = f.read()

    header_end = raw.find(b"DATA binary")
    is_ascii = header_end == -1
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
            if i >= pts_count:
                break
            parts = line.split()
            if len(parts) >= 3:
                xyz[i] = [float(parts[0]), float(parts[1]), float(parts[2])]
    else:
        point_size = sum(sizes)
        x_idx = fields.index("x") * 4
        y_idx = fields.index("y") * 4
        z_idx = fields.index("z") * 4
        for i in range(pts_count):
            off = i * point_size
            xyz[i] = [
                struct.unpack("f", data[off + x_idx : off + x_idx + 4])[0],
                struct.unpack("f", data[off + y_idx : off + y_idx + 4])[0],
                struct.unpack("f", data[off + z_idx : off + z_idx + 4])[0],
            ]

    return xyz, header


def write_pcd_binary(path, xyz, header):
    pts_count = len(xyz)
    new_header = re.sub(r"POINTS \d+", f"POINTS {pts_count}", header)
    new_header = re.sub(r"WIDTH \d+", f"WIDTH {pts_count}", new_header)
    # header 本身不含 "DATA binary" 行，直接确保末尾换行即可
    if not new_header.endswith("\n"):
        new_header += "\n"

    with open(path, "wb") as f:
        f.write(new_header.encode("utf-8"))
        f.write(b"DATA binary\n")
        for i in range(pts_count):
            f.write(struct.pack("fff", xyz[i, 0], xyz[i, 1], xyz[i, 2]))


# ======================== 变换 ========================
def apply_transform(xyz, state):
    """返回变换后的 XY 坐标 (不修改原始数据)"""
    xy = xyz[:, :2].copy()
    xy *= state["scale"]
    th = np.deg2rad(state["theta"])
    c, sn = np.cos(th), np.sin(th)
    R = np.array([[c, -sn], [sn, c]])
    xy = xy @ R.T
    xy[:, 0] += state["dx"]
    xy[:, 1] += state["dy"]
    return xy


def get_display_mask(xyz, state):
    """根据 Z 过滤条件返回哪些点应该显示"""
    z_min, z_max = state["z_min"], state["z_max"]
    return (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)


def sample_for_display(xy, max_pts=DISPLAY_MAX_PTS):
    """随机降采样以加速渲染"""
    if len(xy) <= max_pts:
        return xy
    idx = np.random.choice(len(xy), max_pts, replace=False)
    return xy[idx]


# ======================== 主程序 ========================
def main():
    print("=" * 60)
    print("  3D PCD → 2D Map 手动校准工具")
    print("=" * 60)
    print(f"  场景:     {SCENE}")
    print(f"  2D 地图:  {MAP_2D_PATH}")
    print(f"  3D PCD:   {PCD_PATH}")
    print(f"  场地尺寸:  {FIELD_W} x {FIELD_H} m")
    print()
    print("  [提示] 2D 地图原点在左下角 (X→右, Y→上)")
    print("  [提示] 3D PCD 的 (0,0) 可能不在左下角，需要手动对齐")
    print("=" * 60)
    print("  按 H 显示操作帮助")
    print()

    # ---- 加载数据 ----
    for path, name in [(MAP_2D_PATH, "2D 地图"), (PCD_PATH, "3D PCD")]:
        if not os.path.exists(path):
            print(f"[ERROR] {name}不存在: {path}")
            sys.exit(1)

    img = mpimg.imread(MAP_2D_PATH)
    pcd_xyz, pcd_header = read_pcd_binary(PCD_PATH)

    print(f"  2D 地图: {img.shape[1]} x {img.shape[0]} px")
    print(f"  3D PCD:  {len(pcd_xyz)} 点")
    print(f"  PCD X:   [{pcd_xyz[:,0].min():.2f}, {pcd_xyz[:,0].max():.2f}]  span={pcd_xyz[:,0].max()-pcd_xyz[:,0].min():.2f}m")
    print(f"  PCD Y:   [{pcd_xyz[:,1].min():.2f}, {pcd_xyz[:,1].max():.2f}]  span={pcd_xyz[:,1].max()-pcd_xyz[:,1].min():.2f}m")
    print(f"  PCD Z:   [{pcd_xyz[:,2].min():.2f}, {pcd_xyz[:,2].max():.2f}]  span={pcd_xyz[:,2].max()-pcd_xyz[:,2].min():.2f}m")
    print()

    # ---- 变换状态 ----
    state = {
        "dx": 0.0,
        "dy": 0.0,
        "theta": 0.0,    # 度
        "scale": 1.0,
        "alpha": 0.5,
        "z_min": -2.0,   # Z 过滤下限
        "z_max": 3.0,    # Z 过滤上限
    }

    # 自动预平移：如果 PCD 有大片负坐标区域，自动将原点移到场地左下
    y_min = pcd_xyz[:, 1].min()
    x_min = pcd_xyz[:, 0].min()
    auto_dx, auto_dy = 0.0, 0.0
    if x_min < -0.5:
        auto_dx = -x_min
    if y_min < -0.5:
        auto_dy = -y_min
    if auto_dx != 0.0 or auto_dy != 0.0:
        state["dx"] = auto_dx
        state["dy"] = auto_dy
        print(f"  [自动] 计算初始平移: dx={auto_dx:.2f}, dy={auto_dy:.2f}")
        print(f"         使 PCD 的 min_x/min_y 对齐到 0")
        print()

    # ---- 绘图 ----
    fig, ax = plt.subplots(1, 1, figsize=(14, 8))
    fig.canvas.manager.set_window_title("3D PCD → 2D Map 校准 — 按 H 查看帮助")

    # 2D 地图背景 (origin="upper" 配合 extent 实现 Y 轴翻转，原点在左下)
    ax.imshow(img, extent=[0, FIELD_W, 0, FIELD_H],
              origin="upper", aspect="auto", alpha=0.85)

    # 初始化 PCD scatter
    def get_display_xy():
        mask = get_display_mask(pcd_xyz, state)
        xy_full = apply_transform(pcd_xyz[mask], state)
        return sample_for_display(xy_full)

    display_xy = get_display_xy()
    scatter = ax.scatter(
        display_xy[:, 0], display_xy[:, 1],
        c="#ff4444", s=0.5, alpha=state["alpha"],
        label="3D PCD (俯视投影)",
    )

    # 场地边界
    rect = plt.Rectangle((0, 0), FIELD_W, FIELD_H, fill=False,
                          edgecolor="lime", linewidth=2, linestyle="--",
                          label=f"场地边界 ({FIELD_W}x{FIELD_H}m)")
    ax.add_patch(rect)

    # 标注
    ax.set_xlim(-3, FIELD_W + 3)
    ax.set_ylim(-3, FIELD_H + 3)
    ax.set_xlabel("X (m)  →  红方 → 蓝方", fontsize=11)
    ax.set_ylabel("Y (m)  →  下方 → 上方", fontsize=11)
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.2)

    # 场地中心十字
    ax.axhline(y=FIELD_H/2, color="gray", alpha=0.3, lw=0.5)
    ax.axvline(x=FIELD_W/2, color="gray", alpha=0.3, lw=0.5)
    # 标注红蓝方
    ax.text(-1.5, FIELD_H/2, "RED\nSIDE", ha="center", va="center",
            fontsize=8, color="red", alpha=0.5, fontweight="bold")
    ax.text(FIELD_W + 1.5, FIELD_H/2, "BLUE\nSIDE", ha="center", va="center",
            fontsize=8, color="blue", alpha=0.5, fontweight="bold")

    # ---- 状态栏 ----
    def make_title(s):
        z_info = f"Z∈[{s['z_min']:.1f}, {s['z_max']:.1f}]"
        return (
            f"dx={s['dx']:7.2f}  dy={s['dy']:7.2f}  θ={s['theta']:7.1f}°  "
            f"scale={s['scale']:.3f}  α={s['alpha']:.2f}  {z_info}\n"
            f"W/A/S/D:平移  Shift+W/A/S/D:微调  Q/E:旋转  Z/X:Z过滤  1/2:透明度  R:重置  Enter:保存  H:帮助"
        )

    ax.set_title(make_title(state), fontsize=9, family="monospace")

    # ---- 更新显示 ----
    def update_display():
        xy = get_display_xy()
        scatter.set_offsets(xy)
        scatter.set_alpha(state["alpha"])
        ax.set_title(make_title(state), fontsize=9, family="monospace")
        fig.canvas.draw_idle()

    # ---- 键盘回调 ----
    STEP_XY      = 0.2
    STEP_XY_FINE = 0.05
    STEP_ROT      = 1.0
    STEP_ROT_FINE = 0.1

    def on_key(event: KeyEvent):
        nonlocal state
        key = event.key
        if key is None:
            return

        changed = True
        fine = event.key.startswith("shift+") or (
            hasattr(event, "guiEvent") and event.guiEvent is not None
            and hasattr(event.guiEvent, "modifiers")
        )

        # 检测 Shift 修饰
        shift = False
        try:
            if hasattr(event, "guiEvent") and event.guiEvent is not None:
                shift = bool(event.guiEvent.modifiers() & matplotlib.backend_bases.KeyModifier.SHIFT)
        except Exception:
            pass

        step_xy = STEP_XY_FINE if shift else STEP_XY
        step_rot = STEP_ROT_FINE if shift else STEP_ROT

        if key in ("w", "W"):
            state["dy"] += step_xy
        elif key in ("s", "S"):
            state["dy"] -= step_xy
        elif key in ("a", "A"):
            state["dx"] -= step_xy
        elif key in ("d", "D"):
            state["dx"] += step_xy
        elif key in ("q", "Q"):
            state["theta"] += step_rot
        elif key in ("e", "E"):
            state["theta"] -= step_rot
        elif key in ("z", "Z"):
            # Z 过滤: 抬高
            state["z_min"] += 0.2
            state["z_max"] += 0.2
            print(f"  Z 过滤范围: [{state['z_min']:.1f}, {state['z_max']:.1f}] "
                  f"(显示 {state['z_min']:.1f}~{state['z_max']:.1f}m 高度的点)")
        elif key in ("x", "X"):
            # Z 过滤: 降低
            state["z_min"] -= 0.2
            state["z_max"] -= 0.2
            print(f"  Z 过滤范围: [{state['z_min']:.1f}, {state['z_max']:.1f}] "
                  f"(显示 {state['z_min']:.1f}~{state['z_max']:.1f}m 高度的点)")
        elif key == "1":
            state["alpha"] = max(0.05, state["alpha"] - 0.05)
        elif key == "2":
            state["alpha"] = min(1.0, state["alpha"] + 0.05)
        elif key in ("r", "R"):
            state["dx"] = auto_dx
            state["dy"] = auto_dy
            state["theta"] = 0.0
            state["scale"] = 1.0
            state["alpha"] = 0.5
            state["z_min"] = -2.0
            state["z_max"] = 3.0
            print("[重置] 所有变换已恢复默认")
        elif key in ("h", "H"):
            print_help()
            changed = False
        elif key == "enter":
            save_result(pcd_xyz, pcd_header, state)
            changed = False
        elif key == "escape":
            print("[退出]")
            plt.close()
            return
        else:
            changed = False

        if changed:
            update_display()

    def print_help():
        print("""
╔══════════════════════════════════════════════════════╗
║              操  作  说  明                           ║
╠══════════════════════════════════════════════════════╣
║  W/A/S/D        平移 PCD (↑←↓→)    步长 0.2m        ║
║  Shift+W/A/S/D  微调平移            步长 0.05m       ║
║  Q/E            旋转 PCD (逆/顺)    步长 1.0°        ║
║  Shift+Q/E      微调旋转            步长 0.1°        ║
║  Z/X            按高度过滤 (仅显示特定 Z 范围的点)    ║
║  1/2            降低/提高点云透明度                   ║
║  R              重置所有变换                         ║
║  Enter          保存变换后的 PCD 并输出矩阵           ║
║  H              显示本帮助                           ║
║  Esc            退出                                 ║
╠══════════════════════════════════════════════════════╣
║  校准思路:                                           ║
║  1. 先用 Z/X 过滤到地面高度 (~0m)，看地面轮廓对齐    ║
║  2. 用 W/A/S/D 平移使地面轮廓对准 2D 地图            ║
║  3. 用 Q/E 微调旋转                                  ║
║  4. 抬高 Z 过滤看墙壁/障碍物是否也对齐               ║
║  5. 按 Enter 保存                                    ║
╚══════════════════════════════════════════════════════╝
        """)

    def save_result(xyz, header, s):
        th = np.deg2rad(s["theta"])
        c, sn = np.cos(th), np.sin(th)

        T = np.eye(4)
        T[0, 0] = s["scale"] * c
        T[0, 1] = -s["scale"] * sn
        T[1, 0] = s["scale"] * sn
        T[1, 1] = s["scale"] * c
        T[0, 3] = s["dx"]
        T[1, 3] = s["dy"]

        # 应用变换
        xyz_aligned = xyz.copy()
        xy = xyz[:, :2] * s["scale"]
        xy_rot = xy @ np.array([[c, sn], [-sn, c]])
        xyz_aligned[:, 0] = xy_rot[:, 0] + s["dx"]
        xyz_aligned[:, 1] = xy_rot[:, 1] + s["dy"]

        print("\n" + "=" * 60)
        print("  校准结果")
        print("=" * 60)
        print(f"  平移 dx:  {s['dx']:.4f} m")
        print(f"  平移 dy:  {s['dy']:.4f} m")
        print(f"  旋转 θ:   {s['theta']:.4f} °")
        print(f"  缩放:     {s['scale']:.6f}")
        print()
        print("  4x4 变换矩阵 T (P_new = T * P_original):")
        print(f"  ┌ {'':>10} {'':>10} {'':>10} {'':>10} ┐")
        print(f"  │ {T[0,0]:10.6f}  {T[0,1]:10.6f}  {T[0,2]:10.6f}  {T[0,3]:10.4f} │")
        print(f"  │ {T[1,0]:10.6f}  {T[1,1]:10.6f}  {T[1,2]:10.6f}  {T[1,3]:10.4f} │")
        print(f"  │ {T[2,0]:10.6f}  {T[2,1]:10.6f}  {T[2,2]:10.6f}  {T[2,3]:10.4f} │")
        print(f"  │ {T[3,0]:10.6f}  {T[3,1]:10.6f}  {T[3,2]:10.6f}  {T[3,3]:10.4f} │")
        print(f"  └ {'':>10} {'':>10} {'':>10} {'':>10} ┘")
        print()
        print(f"  变换前 PCD X: [{xyz[:,0].min():.2f}, {xyz[:,0].max():.2f}]")
        print(f"  变换前 PCD Y: [{xyz[:,1].min():.2f}, {xyz[:,1].max():.2f}]")
        print(f"  变换后 PCD X: [{xyz_aligned[:,0].min():.2f}, {xyz_aligned[:,0].max():.2f}]")
        print(f"  变换后 PCD Y: [{xyz_aligned[:,1].min():.2f}, {xyz_aligned[:,1].max():.2f}]")

        write_pcd_binary(OUTPUT_PCD, xyz_aligned, header)
        print(f"\n  ✓ 已保存到: {OUTPUT_PCD}")
        print("=" * 60 + "\n")

        # 也输出 main_config.yaml 中需要改的 PCD 路径建议
        print("  [提示] 如需使用对齐后的 PCD，请修改 main_config.yaml:")
        print(f"    scenes.lab.pcd_file: \"source/pointclouds/background/background2_aligned.pcd\"")
        print(f"    scenes.lab.downsampled_pcd: \"source/pointclouds/background/background2_aligned.pcd\"")
        print()

    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
