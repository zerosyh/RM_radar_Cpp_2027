# HNURM Radar 2027

湖南大学 RoboMaster 2027 雷达站项目（C++ 实现）。

基于激光雷达（Livox HAP）和单目相机（海康），通过点云背景减除、TensorRT 三阶段神经网络推理、ICP 点云配准和卡尔曼滤波融合，实现地面机器人和空中无人机的实时检测、跟踪与定位。

---

## 架构

```
hik_camera ──共享内存──► detect ──detect_result──────┐
                         │ 3-stage TensorRT           │
livox ──► lidar ──target_pointcloud──► radar ──location──► display_panel
          │ 背景减除     │               │ KF 融合       │  小地图
          │ 无人机区域    │               │ 无人机检测     │
          └─livox/lidar_other──────────┘               │
          └─lidar_pcds──► registration ──TF──┘
                          ICP 配准
```

### 数据流

| 节点 | 订阅 | 发布 | 功能 |
|------|------|------|------|
| `hik_camera_node` | — | 共享内存 + `image_raw` | 海康相机驱动（3072×2048 @ 60fps） |
| `detect` | 共享内存 / rosbag / USB | `detect_result` | 三阶段 YOLO 推理 → 车辆识别 + 数字分类 |
| `lidar` | `/livox/lidar` | `target_pointcloud`, `lidar_pcds`, `livox/lidar_other` | 背景减除 + 动态点云提取 + 无人机区域筛选 |
| `registration` | `lidar_pcds` | TF `map→livox_frame` | Quatro 粗配准 + small_gicp 精配准 |
| `radar` | `target_pointcloud` + `detect_result` + `livox/lidar_other` | `location` | DBSCAN 聚类 + KF 融合 + 无人机跟踪 |
| `display_panel` | `location` | `/map_view` | 场地小地图可视化 |

---

## 硬件依赖

| 设备 | 型号 |
|------|------|
| 激光雷达 | Livox HAP |
| 单目相机 | Hikvision CH-120-10UC |
| GPU | NVIDIA RTX 5060 (Compute Capability 12.0) |
| CPU | Intel i7 或以上 |

---

## 软件依赖

```
Ubuntu 22.04
ROS2 Humble
CUDA + TensorRT 10.x
OpenCV 4.x
PCL 1.12+
Eigen3
yaml-cpp
nlohmann-json
Livox-SDK2
small_gicp + Quatro + TEASER++
海康 MVS SDK
```

---

## 快速开始

### 1. 环境准备

```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash

# TensorRT 库路径
export LD_LIBRARY_PATH=/path/to/TensorRT/lib:$LD_LIBRARY_PATH

# 移除 MVS 库路径（避免 libusb 冲突）
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|/opt/MVS/lib/64:||g')
```

### 2. 编译

```bash
cd ~/rm_lidar_2027/RM_radar_Cpp_2027
colcon build
source install/setup.bash
```

### 3. 配置

编辑 `configs/main_config.yaml`：

```yaml
global:
  my_color: "Blue"           # 己方颜色: "Red" / "Blue"
  scene: "competition"       # 场景: "competition" / "lab"
  debug_coordinate_publish: true  # true = 调试模式（发布所有轨迹）
                                  # false = 赛场模式（仅发布敌方+己方哨兵）
camera:
  mode: "rosbag"             # "hik" / "rosbag" / "video" / "test"
```

### 4. 启动

使用 `bringup.sh` 一键启动全部节点（需要 gnome-terminal）：

```bash
bash bringup.sh
```

或手动启动各节点：

```bash
# 终端1: 雷达驱动 + RViz
ros2 launch livox_ros_driver2 rviz_HAP_launch.py

# 终端2: 视觉检测
ros2 run hnurm_radar detect

# 终端3: 背景减除
ros2 run hnurm_radar lidar

# 终端4: 点云配准
ros2 launch registration registration.launch.py

# 终端5: 融合 + 无人机
ros2 run hnurm_radar radar

# 终端6: 可视化
ros2 run hnurm_radar display_panel

# 终端7: rosbag 回放（可选）
ros2 bag play <rosbag_path> --rate 1.0
```

---

## 检测原理

### 地面机器人

```
Stage1: 1280×1280 整车检测 (YOLO, ~17ms)
  → 裁剪车辆 ROI
Stage2: 192×192 装甲板检测 + 颜色分类 (YOLO, ~7ms)
  → 裁剪装甲板 ROI
Stage3: 64×64 数字分类 (ResNet, ~1ms)
  → 输出标签 "R1", "B3", "RS" 等
```

- **颜色/数字稳定**：滑动窗口（7帧）多数投票，消除单帧突变
- **轨迹跟踪**：IoU 贪心匹配 + 卡尔曼滤波器，连续 miss ≥ 5 帧才清理

### 激光雷达

- 背景减除：预加载 PCD 地图建立 kd-tree，实时点云最近邻距离 > 阈值判定为动态
- 场地分区：过滤场外区域、环高、飞镖区，区分主赛场和无人机走廊
- 多帧累积：5 帧点云叠加提高密度

### 传感器融合

```
detectCb (相机帧率) ──► cameraMatch   → 填充 detect_history
pcdCb   (雷达帧率) ──► predict → DBSCAN匹配 → KF→correct → 清理 → 发布
```

- 相机和雷达各自独立运行，通过卡尔曼滤波融合
- **雷达无点时仍做 predict + publish**，纯视觉 KF 不被雷达锁死
- 颜色/数字由 KF 历史多数投票确定，unknown 不投票，平票取最近一票

### 空中无人机

```
livox/lidar_other
  → DBSCAN 聚类 (eps=0.3, min_samples=5)
  → KF 跟踪 (max_speed=10m/s)
  → hits≥3 稳定后发布
  → 位置判定颜色: x<14→Red(6), x≥14→Blue(106)
  → per-KF 迟滞死区 (x∈[13.5,14.5] 不动)
```

- 双方无人机区域对称覆盖
- 与地面轨迹合并到同一 `location` 消息发布

---

## ROS2 话题

### 订阅

| 话题 | 类型 | 节点 | 说明 |
|------|------|------|------|
| `/livox/lidar` | PointCloud2 | lidar | Livox 原始点云 |
| `target_pointcloud` | PointCloud2 | radar | 动态目标点云（地面） |
| `livox/lidar_other` | PointCloud2 | radar | 无人机区域点云 |
| `detect_result` | Robots | radar | 视觉检测结果 |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `detect_result` | Robots | 检测框 + 标签 + 场地坐标 |
| `location` | Locations | 融合后的机器人位置（地面 + 空中） |
| `lidar_pcds` | PointCloud2 | 累积原始点云（供配准用） |
| `/map_view` | Image | 小地图缩略图 |

---

## 配置说明

| 配置文件 | 用途 |
|----------|------|
| `configs/main_config.yaml` | 全局配置（颜色/场景/相机/雷达/初始位姿） |
| `configs/detector_config.yaml` | 检测参数（模型路径/置信度/标签/滤波） |
| `configs/converter_config.yaml` | 相机-雷达外参（实时模式） |
| `configs/converter_config_rosbag.yaml` | 相机-雷达外参（rosbag 模式） |
| `configs/perspective_calib.json` | 透视变换标定矩阵 |

### 关键参数

```yaml
# 检测
params:
  labels: ["1","2","3","4","S","Q"]   # 数字分类标签
  stage_one_conf: 0.60                # 整车检测置信度阈值
  stage_two_conf: 0.5                 # 装甲板检测置信度阈值

# 滤波
filter:
  max_inactive_time: 3.0   # KF 超时清理时间 (s)
  max_velocity: 2.5         # 地面最大速度 (m/s)
  jump_threshold: 1.0       # 单帧跳变阈值 (m)

# 雷达
lidar:
  min_distance: 1           # 最小距离 (m)
  max_distance: 40          # 最大距离 (m)
  lidar_topic_name: "/livox/lidar"

# 配准初始位姿（引导 ICP 收敛方向）
initial_pose:
  red:
    x: -1.69    y: 4.23    z: 4.0    yaw: 0.21
  blue:
    x: 29.66    y: 10.61   z: 4.0    yaw: -2.94
```

---

## 项目结构

```
RM_radar_Cpp_2027/
├── bringup.sh                    # 一键启动脚本
├── README.md
├── configs/
│   ├── main_config.yaml          # 全局配置
│   ├── detector_config.yaml      # 检测参数
│   ├── converter_config.yaml     # 相机-雷达外参
│   ├── converter_config_rosbag.yaml
│   └── perspective_calib.json    # 透视标定
├── model/
│   ├── ONNX/                     # ONNX 模型
│   └── TensorRT/                 # TensorRT 引擎文件
│       ├── car.engine            # 整车检测
│       ├── armor2_hku.engine     # 装甲板检测
│       └── digit_hku.engine      # 数字分类
├── source/
│   ├── maps/                     # 场地地图图像
│   └── pointclouds/              # PCD 点云文件
│       ├── registration/         # 配准用全局地图
│       └── background/           # 背景减除用地图
├── src/
│   ├── hnurm_radar/              # 主雷达包
│   │   ├── src/
│   │   │   ├── detect.cpp        # 视觉检测节点
│   │   │   ├── infer.cu          # CUDA 推理实现
│   │   │   ├── radar.cpp         # 融合 + 无人机节点
│   │   │   ├── lidar.cpp         # 激光雷达处理
│   │   │   ├── display_panel.cpp # 小地图显示
│   │   │   ├── hik_camera_node.cpp  # 海康相机驱动
│   │   │   ├── HomographyTransformer.cpp  # 透视变换
│   │   │   ├── tracker.cpp       # 帧间追踪器
│   │   │   └── hnurm_radr_node.cpp  # [占位]
│   │   ├── include/hnurm_radar/  # 头文件
│   │   └── CMakeLists.txt
│   ├── registration/             # 点云配准
│   ├── detect_result/            # 自定义 ROS2 消息
│   ├── livox_ros_driver2/        # Livox ROS2 驱动
│   ├── Livox-SDK2/               # Livox SDK
│   └── Quatro/                   # Quatro 配准算法
└── log/                          # 运行日志
```

---

## 开发说明

- **GPU 内存**：3 个 scratch buffer 均已预分配（car/armor/digit），无运行时动态分配
- **线程安全**：`KFs_` 和 `KFs_air_` 分别由独立 mutex 保护
- **KF 生命力**：雷达无点时 KF 仍做 predict，不会冻结
- **坐标对齐**：雷达通过 registration TF 变换到 map 坐标，相机通过透视变换到场地坐标，两者需联合标定对齐原点

## 参考

- [T-DT 2025 Radar](https://github.com/T-DT-Algorithm-2024/T-DT-2024-Radar) — 东北大学雷达站
- [HNURM Radar 2026](../HNURM-radar-2026/) — 湖南大学 2026 Python 版雷达站

## License

MIT
