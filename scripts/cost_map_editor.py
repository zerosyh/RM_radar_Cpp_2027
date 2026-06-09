#!/usr/bin/env python3
"""
cost_map_editor.py - 在场地地图上编辑哨兵代价栅格

左键=障碍(黑)  右键=坡道(灰)  中键=清除(白)  关闭=保存
"""

import cv2
import numpy as np
import os

GRID_W, GRID_H = 56, 30    # 0.5m 分辨率
CELL_SIZE = 30
BG_ALPHA = 0.40   # 底图透明度

COLOR_FLAT  = (255, 255, 255)
COLOR_SLOPE = (130, 130, 130)
COLOR_WALL  = (30, 30, 30)

grid = np.full((GRID_H, GRID_W), 0, dtype=np.int8)

# 预置
presets = [
    (0, 0, 2, 1, 2),       # 红方补给站
    (25, 13, 27, 14, 2),    # 蓝方补给站
    (17, 6, 19, 7, 2),      # 环高柱区
    (16, 2, 21, 4, 1),      # 下坡
    (16, 10, 21, 12, 1),    # 上坡
]
for x1, y1, x2, y2, t in presets:
    for y in range(y1, y2 + 1):
        for x in range(x1, x2 + 1):
            if 0 <= x < GRID_W and 0 <= y < GRID_H:
                grid[y, x] = t

# 加载地图背景
script_dir = os.path.dirname(os.path.abspath(__file__))
proj_root = os.path.normpath(os.path.join(script_dir, ".."))
map_path = os.path.join(proj_root, "source", "maps", "competition_2026", "std_map.png")
bg = cv2.imread(map_path)
if bg is None:
    alt = os.path.join(proj_root, "configs", "icp.rviz")
    bg = np.full((GRID_H * CELL_SIZE, GRID_W * CELL_SIZE, 3), 240, dtype=np.uint8)
    print(f"WARNING: {map_path} not found, using blank background")
else:
    bg = cv2.resize(bg, (GRID_W * CELL_SIZE, GRID_H * CELL_SIZE))
    print(f"Loaded: {map_path}")

MARGIN = 10
canvas_w = GRID_W * CELL_SIZE + MARGIN * 2
canvas_h = GRID_H * CELL_SIZE + MARGIN * 2 + 30

def grid_cell(gx, gy):
    return (MARGIN + gx * CELL_SIZE, MARGIN + gy * CELL_SIZE,
            MARGIN + gx * CELL_SIZE + CELL_SIZE, MARGIN + gy * CELL_SIZE + CELL_SIZE)

def draw():
    canvas = np.full((canvas_h, canvas_w, 3), 50, dtype=np.uint8)
    gw = GRID_W * CELL_SIZE
    gh = GRID_H * CELL_SIZE

    # 底图半透明
    bg_canvas = canvas.copy()
    bg_canvas[MARGIN:MARGIN + gh, MARGIN:MARGIN + gw] = bg
    canvas = cv2.addWeighted(bg_canvas, BG_ALPHA, canvas, 1 - BG_ALPHA, 0)

    # 栅格 (全不透明)
    for gy in range(GRID_H):
        for gx in range(GRID_W):
            x1, y1, x2, y2 = grid_cell(gx, gy)
            if grid[gy, gx] == 2:
                cv2.rectangle(canvas, (x1, y1), (x2, y2), COLOR_WALL, -1)
            elif grid[gy, gx] == 1:
                cv2.rectangle(canvas, (x1, y1), (x2, y2), COLOR_SLOPE, -1)
    # 网格线 (细)
    for gy in range(GRID_H + 1):
        cv2.line(canvas, (MARGIN, MARGIN + gy * CELL_SIZE),
                 (MARGIN + gw, MARGIN + gy * CELL_SIZE), (60, 60, 60), 1)
    for gx in range(GRID_W + 1):
        cv2.line(canvas, (MARGIN + gx * CELL_SIZE, MARGIN),
                 (MARGIN + gx * CELL_SIZE, MARGIN + gh), (60, 60, 60), 1)
    # 坐标
    for gy in range(GRID_H):
        cv2.putText(canvas, str(gy), (2, MARGIN + gy * CELL_SIZE + CELL_SIZE // 2 + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (180, 180, 180), 1)
    for gx in range(GRID_W):
        cv2.putText(canvas, str(gx), (MARGIN + gx * CELL_SIZE + CELL_SIZE // 2 - 8,
                    MARGIN + GRID_H * CELL_SIZE + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (180, 180, 180), 1)
    # 图例
    legend = "L=obstacle  R=slope  M=clear  Q/close=save  cost_map.png"
    cv2.putText(canvas, legend, (MARGIN, canvas_h - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
    return canvas

def mouse_cb(event, x, y, flags, param):
    gx = (x - MARGIN) // CELL_SIZE
    gy = (y - MARGIN) // CELL_SIZE
    if gx < 0 or gx >= GRID_W or gy < 0 or gy >= GRID_H:
        return
    if event == cv2.EVENT_LBUTTONDOWN:
        grid[gy, gx] = 2
    elif event == cv2.EVENT_RBUTTONDOWN:
        grid[gy, gx] = 1
    elif event == cv2.EVENT_MBUTTONDOWN:
        grid[gy, gx] = 0

cv2.namedWindow("CostMap Editor", cv2.WINDOW_NORMAL)
cv2.resizeWindow("CostMap Editor", canvas_w, canvas_h + 40)
cv2.setMouseCallback("CostMap Editor", mouse_cb)

while True:
    cv2.imshow("CostMap Editor", draw())
    key = cv2.waitKey(100) & 0xFF
    if key == ord('q') or cv2.getWindowProperty("CostMap Editor", cv2.WND_PROP_VISIBLE) < 1:
        break

cv2.destroyAllWindows()

# 保存 28x15 PNG
out = np.zeros((GRID_H, GRID_W, 3), dtype=np.uint8)
for gy in range(GRID_H):
    for gx in range(GRID_W):
        v = grid[gy, gx]
        out[gy, gx] = COLOR_WALL if v == 2 else (COLOR_SLOPE if v == 1 else COLOR_FLAT)

out_path = os.path.join(proj_root, "configs", "cost_map.png")
cv2.imwrite(out_path, out)
print(f"Saved: {out_path} ({GRID_W}x{GRID_H})")
print("Grid: 0=flat 1=slope 2=wall")
print(grid)
