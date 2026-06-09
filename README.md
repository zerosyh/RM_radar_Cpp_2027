# HNURM Radar 2027

湖南大学 RoboMaster 2027 雷达站项目（C++ 实现）。

激光雷达 + 单目相机 → 3 阶段 TensorRT 推理 → KF 融合 → 实时定位 → 哨兵辅助决策。

---

## 架构

```
detect ──detect_result──┐
  │ 3-stage TRT + 时序过滤  │
lidar ──target_pointcloud──→ radar ──location──→ display_panel
  │ 背景减除 + 无人机区域     │ KF融合 + 无人机 + last_known  │ 地图 + 哨兵标记
  └───livox/lidar_other──┘                              └───sentry_targets──┘
                                                              ↑
registration ──TF──→ map                              sentry_decision
  Quatro + small_gicp                                    攻防决策 + 威胁评估
```

## 节点

| 节点 | 行 | 功能 |
|------|:--:|------|
| `detect` | 633 | 3-stage TRT (1280²→192²→64²) + 7帧 TrackHistory 多数投票 |
| `lidar` | 317 | kd-tree 背景减除 + 双方无人机区域筛选 |
| `radar` | 713 | KF 融合 (T-DT 对齐) + 无人机 KF + last_known 轨迹 + 统一发布 |
| `display_panel` | 279 | 场地小地图 + 哨兵标记绘制 |
| `sentry_decision` | 211 | 攻防模式判定 + 威胁评估 + 攻击/警戒/导航目标 |
| `hik_camera` | 397 | 海康 MVS 驱动 + 共享内存 |
| `registration` | 789 | Quatro + small_gicp 配准 → TF: map→livox_frame |

---

## 硬件

| 设备 | 型号 |
|------|------|
| 激光雷达 | Livox HAP |
| 相机 | Hikvision CH-120-10UC |
| GPU | RTX 5060+ (CC 12.0) |

## 软件

`Ubuntu 22.04` `ROS2 Humble` `CUDA + TensorRT 10.x` `OpenCV 4` `PCL 1.12` `Eigen3` `yaml-cpp` `nlohmann-json` `Livox-SDK2` `small_gicp + Quatro + TEASER++` `海康 MVS SDK`

---

## 快速开始

```bash
# 编译
cd RM_radar_Cpp_2027
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|/opt/MVS/lib/64:||g')
export LD_LIBRARY_PATH=/path/to/TensorRT/lib:$LD_LIBRARY_PATH
colcon build && source install/setup.bash

# 启动
bash bringup.sh

# 或手动:
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
ros2 run hnurm_radar detect
ros2 run hnurm_radar lidar
ros2 launch registration registration.launch.py
ros2 run hnurm_radar radar
ros2 run hnurm_radar sentry_decision
ros2 run hnurm_radar display_panel
```

## 配置

`configs/main_config.yaml`:

```yaml
global:
  my_color: "Blue"
  scene: "competition"
  debug_coordinate_publish: true
camera:
  mode: "rosbag"
sentry:              # 哨兵决策 (可选)
  x: 27.0  y: 7.5
```

---

## 检测原理

### 地面机器人 — 3 阶段 TensorRT

```
Stage1: 1280² 整车 YOLO (~17ms, 300x6)  → 裁剪 192² ROI
Stage2: 192² 装甲板 YOLO (~7ms)          → cls: 0=dead 1=Red 2=Blue
Stage3: 64² 数字 ResNet (~1ms)           → labels_[0..5] = [1,2,3,4,S,Q]

TrackHistory: 每 track_id 7帧滑动窗口多数投票颜色+数字
  BGR 通道差分逐帧判定 → 颜色帧级投票
  miss ≥ 5帧才删历史
```

### 激光雷达

```
raw → 距离过滤(1-40m) → TF→map → 场地分区
  main: → kd-tree 背景减除(0.15m) → 3帧累积 → target_pointcloud
  other: 双方无人机区域 → livox/lidar_other
```

### KF 融合 (对齐 T-DT)

```
pcdCb (10Hz):
  Step 1: predict (固定 dt=0.1)
  Step 2: DBSCAN(eps=0.15) → match(dt上限0.3s) → KF.correct → history
  Step 3: last_time>2s → 保存 last_known(-cid, 10s) → 清理
  Step 4: best_per_cid 去重 + clamp + last_known(live覆盖跳过)
  Step 5: 追加无人机

detectCb: cameraMatch → history时间对齐(<1s) + 空间匹配 → detect_history
getColor: R/B 多数投票, 平票取最后, D不投
getNumber: 当选颜色数字票多数
```

### 无人机

```
区域: Blue x∈[13.5,27],y∈[0.5,4.5] | Red x∈[2,14.5],y∈[10.5,14.3]
DBSCAN(eps=0.3,min=5) → KF(max_speed=10m/s) → hits≥3发布
颜色: x<13.5→Red(6), x>14.5→Blue(106), [13.5,14.5]→per-KF迟滞
```

### 哨兵决策

```
判定: 敌军在我方半场→DEFENSE(黄) | 敌方半场→OFFENSE(红)

DEFENSE: 攻击=最近敌, 导航=拦截位(中点)
OFFENSE: 攻击=最高威胁(type/dist), 导航=向目标靠至5m射程
威胁权重: Hero=5, Eng=3, Inf=2, Sentry=1, Outpost=0.5

输出: ATTACK(⊕十字) ALERT(→箭头) SENTRY(●白点) NAV(◆绿菱) DEF/ATK(模式标签)
```

---

## 话题

| 话题 | 方向 | 说明 |
|------|:--:|------|
| `/livox/lidar` | lidar← | Livox 原始点云 |
| `target_pointcloud` | radar← | 地面动态点云 |
| `livox/lidar_other` | radar← | 无人机区域点云 |
| `detect_result` | radar← | 视觉检测结果 |
| `location` | radar→ | 融合位置 (地面+无人机+last_known) |
| `sentry_targets` | sentry→ | 哨兵决策标记 |
| `lidar_pcds` | reg← | 累积点云 (配准用) |

## QoS

| topic | 发布端 | 订阅端 |
|-------|--------|--------|
| `location` | radar reliable | display_panel reliable + sentry_decision best_effort |
| `sentry_targets` | sentry_decision reliable | display_panel reliable |

---

## 项目结构

```
RM_radar_Cpp_2027/
├── bringup.sh
├── README.md
├── configs/
├── model/{ONNX,TensorRT}/
├── source/{maps,pointclouds}/
├── src/
│   ├── hnurm_radar/src/
│   │   ├── detect.cpp            # 视觉检测
│   │   ├── infer.cu              # CUDA 推理
│   │   ├── radar.cpp             # KF 融合 + 无人机
│   │   ├── lidar.cpp             # 激光雷达处理
│   │   ├── display_panel.cpp     # 小地图可视化
│   │   ├── sentry_decision.cpp   # 哨兵决策
│   │   ├── hik_camera_node.cpp   # 相机驱动
│   │   ├── tracker.cpp           # 帧间追踪
│   │   └── HomographyTransformer.cpp
│   ├── registration/             # ICP 配准
│   ├── detect_result/            # 自定义消息
│   ├── livox_ros_driver2/        # Livox 驱动
│   └── Livox-SDK2/ Quatro/
└── log/
```

## 参考

- [T-DT 2025 Radar](https://github.com/T-DT-Algorithm-2024/T-DT-2024-Radar)
- [HNURM Radar 2026](../HNURM-radar-2026/)

## License

MIT
