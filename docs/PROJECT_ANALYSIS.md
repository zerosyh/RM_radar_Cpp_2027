# RM_radar_Cpp_2027 系统分析报告

> 分析日期：2026-08-03
> 依据：全项目源码精读（hnurm_radar / registration / detect_result / scripts / configs / 启动脚本）+ git 历史 + 已有 KNOWN_ISSUES.md 复核
> 符号约定：🔴 高 | 🟡 中 | 🟢 低 | ✅ 已验证 | 📋 源码推断 | ✨ 本次新发现（KNOWN_ISSUES 未收录）

---

## 一、项目概述

湖南大学 RoboMaster 2027 雷达站（C++/ROS2 Humble 实现）：激光雷达 + 单目相机 → 3 阶段 TensorRT 推理 → KF 融合 → 实时定位（/location）→ 哨兵辅助决策（/sentry_targets）→ 裁判系统串口上报（0x0305 坐标 + 0x0301 双倍易伤）。

- 硬件：Livox HAP（激光雷达）、Hikvision CH-120-10UC（3072×2048 BayerRG8）、RTX 5060+（CC 12.0）
- 软件栈：ROS2 Humble / CUDA+TensorRT 10.x / OpenCV 4 / PCL 1.12 / small_gicp + Quatro + TEASER++ / Livox-SDK2 / MVS SDK
- 前身：HNURM-radar-2026（Python）；设计对齐：T-DT 2025 Radar

### 关键设计决策（来自代码事实）

1. **坐标系**：PCD 地图 = 裁判系统坐标系（红方补给站为原点，X 朝蓝 28m，Y 朝红方停机坪 15m）。`main_config.yaml:1-6`
2. **ID 约定**：R1-R7 → 1-7；B1-B7 → 101-107；`debug_coordinate_publish=true` 时全量发布，**比赛必须 false**（仅敌军 + 己方哨兵 7/107）。radar.cpp:569-579
3. **颜色来源**：detect 丢弃 Stage2 模型颜色输出，改用装甲板 ROI 的 BGR 通道均值差分 + 7 帧多数投票（设计选择，2.5 详述）
4. **雷达点云坐标系**：DBSCAN 在 lidar 系聚类 → 中心点 × T_map_lidar（r2f_）转 map 系 → 匹配/发布均用 map 系
5. **last_known 轨迹**：KF 超时删除前保存最后已知位置（负 id），10s 有效，live KF 覆盖时跳过（radar.cpp:532-556）

---

## 二、架构与数据流

```
                    ┌────────────────────────────────────────────────┐
  Livox HAP ──/livox/lidar──→ [lidar] 距离过滤→TF→分区→kd-tree背景减除→3帧累积
                    │          target_pointcloud / livox/lidar_other   │
                    │                                                   ▼
  海康相机 ──shm──→ [detect] ──detect_result──→ [radar]                │
  (hik/rosbag)     3阶段TRT+TrackHistory        DBSCAN→KF融合→location   │
                    │                             │      │              │
                    │              last_known┘ 无人机KF┘              │
                    │                location ──→ [display_panel]      │
                    │                location ──→ [sentry_decision]    │
                    │                sentry_targets ──→ [display_panel]│
                    │                location ──→ [judge_messager]→串口 │
                    ▼                                                   ▼
              [registration] ──TF map→livox_frame──→ 供 lidar/radar 坐标变换
```

### 话题与 QoS 矩阵

| 话题 | 发布端 | 订阅端 | QoS |
|------|--------|--------|-----|
| /livox/lidar | livox_ros_driver2 | lidar | reliable(10) |
| target_pointcloud | lidar | radar | reliable(10) |
| livox/lidar_other | lidar | radar | reliable(10) |
| detect_result | detect | radar | sensor_data(3) |
| location | radar | display_panel / sentry_decision / judge_messager | reliable(10) / best_effort(5) |
| sentry_targets | sentry_decision | display_panel | reliable(5) |
| lidar_pcds | lidar(100Hz 重发缓存) | registration | reliable(10) |
| /global_pcd_map | registration(0.5Hz) | rviz | reliable |
| /map_view | display_panel(每5帧) | 外部查看 | reliable(1) |
| image_raw / /compressed_image | hik / rosbag | detect | sensor_data(3) |

---

## 三、模块级分析

### 3.1 detect（detect.cpp 633 行 + infer.cu 335 行 + tracker.cpp 237 行）

**流水线**：多线程四段式 —— 采集（hik 共享内存轮询 / rosbag 解码线程）→ 处理（单线程，取最新帧丢弃积压）→ 显示 → 发布。

```
原图 ──cv::resize(1280²)──→ Stage1 整车YOLO(conf 0.57) → N×框
  └ 每框 expand×1.2 → Stage2 装甲板YOLO(192², conf 0.5) → 每车M块装甲
      └ 每装甲 expand×1.1 → Stage3 数字ResNet(64²) → labels[1,2,3,4,S,Q]
之后：车辆框→ObjectTracker(IoU 贪心, thr 0.3) → track_id
      装甲板归属到车（中心点落在车内 + IoU 最大化）
      每车取置信度最高装甲板 → BGR差分定色 → 7帧多数投票 → detect_result
      pixelToField(单应变换) → field_x/y
```

**关键事实**：
- Stage1 输出 `300×6 [x1,y1,x2,y2,conf,cls]`，模型内嵌 EfficientNMS，GPU 完成（infer.cu:208-218 仅 conf 过滤 + 钳位）
- Stage2 是**循环单张 enqueue**（伪批量，引擎 batch=1）；Stage3 每块装甲板一次 `cudaStreamSynchronize`（流水线断点）
- 数字置信度（stage3 conf）**未过滤**（`stage_three_conf` 配置未消费）；`probs[max_idx]` 仅作绘制用
- TrackHistory（detect.cpp:556-580）：每 track_id 7 帧滑窗，颜色/数字独立多数投票，miss≥5 帧清历史
- `SimpleKalmanFilter`（tracker.cpp:42）`predict()` 从未被调用 —— 纯死代码，匹配纯 IoU

**耗时基线（README）**：Stage1 ~17ms / Stage2 ~7ms / Stage3 ~1ms

### 3.2 lidar（lidar.cpp 317 行）

```
原始点云 → 距离过滤(1~40m) → 缓存10帧(100Hz重发 lidar_pcds) → TF→map
→ 分区: main(场地内 z<1.4 且非双方停机坪斜线区) / other(无人机区+飞镖区)
→ kd-tree 背景减除(阈值 0.20m) → 3帧累积 → target_pointcloud(转回lidar系)
```

**关键事实**：
- `main_cond`（lidar.cpp:202-206）排除：场外(x<3||x>28||y<0||y>15)、高空(z<0||z>1.4)、双方停机坪(双斜线平行四边形区)
- 无人机区（lidar.cpp:125-136）：Blue x∈[13.5,27]×y∈[0.5,4.5]；Red x∈[2,14.5]×y∈[10.5,14.3]；z∈[1.7,3.5]
- 飞镖区：蓝方堡垒右上角引导灯附近小立方体（isDartRegion）
- 背景地图：**硬编码 `/home/syh/rm_lidar_2027/HNURM-radar-2026/data/pointclouds/background/RM2025.pcd`**，完全忽略 config `lidar.background_map_path` 🔴（KNOWN_ISSUES 1.2）
- 体素降采样 0.1f 硬编码（config `lidar.voxel_size` 未消费）

### 3.3 radar（radar.cpp 717 行）— 系统核心

对齐 T-DT kalman_filter.cpp 的完整实现：

```
pcdCb (10Hz)：
  Step1: 全部 KF predict（固定 dt 取自 get_time()，has_updated 复位）
  Step2: DBSCAN(eps=0.15, min=7) → 聚类中心×T_map_lidar → match(≤max_speed·min(dt,0.3)+r)
         → 单匹配 update / 多匹配取最近 / 无匹配新建 KF
  Step3: last_time>delete_time(3.0) → 有身份则存 last_known(-cid, 10s) → 删 KF
  Step4: 每 KF 发布, 同 cid 取 hits 最大, 位置钳位[-2,30]×[-2,17]
  Step5: append 无人机(livox/lidar_other 回调独立维护 KFs_air_)

detectCb: 立即对全部 KF cameraMatch（时间窗口 1s + 空间半径 detect_r）
身份: parseLabel → getColor(多数投票,平票取最后,D不投) + getNumber(当选颜色数字票多数)
```

**无人机通道**（otherCb）：DBSCAN(eps=0.3, min=5) → KF(max_speed=10, delete_time=3) → hits≥3 发布；颜色迟滞 x<13.5→Red(6)/x>14.5→Blue(106)，死区按 KF 缓存（**以指针为 key，vector 扩容后失效** 🟡 2.8）

**关键事实**：
- KF 噪声 sigma_q=50 / sigma_r=0.1 **硬编码**（radar.cpp:88-89），config `filter.process_noise/measurement_noise` 与 Q/R 无关
- `kf_delete_time_` 默认 1.5f（radar.cpp:287 初始化），config `max_inactive_time=3.0` 覆盖生效
- `detect_cache_`（radar.cpp:296-301, 377-381, 462-469）：只写不读，纯死代码 ✨🟢
- rosbag 模式下订阅 `compressed_image_topic` 但回调为空（radar.cpp:272-276）—— 无实际用途 ✨🟢
- TF 未就绪时 r2f_=Identity 继续发布（radar.cpp:390-392）—— 启动初期会输出 livox 系伪坐标 ✨🟡

### 3.4 display_panel（display_panel.cpp 282 行）

15fps 绘制：场地底图（std_map 缩放 2800×1500）→ 车辆圆形标记（id 大字 + 坐标）→ UAV 菱形 + 高度 → last_known 灰叉 → 哨兵决策标记（SENTRY 白点 / ALERT 箭头 / NAV 绿菱 / DEF/ATK 模式标签）。

**关键事实**：
- `std_map_rel[0]`（display_panel.cpp:31）：空字符串时是 UB（config 恒有值，防御性问题）✨🟢
- 地图路径 `$HOME/rm_lidar_2027/RM_radar_Cpp_2027` 拼接（display_panel.cpp:34）—— **机器路径硬编码** ✨🔴（同 1.2 一类问题）
- PATH 路径线查找 `"ATTACK"/"PATH"` 标签，但 sentry_decision 实际发 `"DEF"/"ATK"` —— 路径线永不绘制 🟡（2.2）
- QoS 不匹配：location 订阅 reliable 而 radar 发布可靠，OK；但 sentry_targets 发布 reliable、此处订阅 reliable，OK（README 记录一致）

### 3.5 sentry_decision（sentry_decision.cpp 186 行）

0.5s 节流处理 location → 哨兵实时位置（自速度估计）→ 五分区攻防判定 → 敌重心（距离倒数加权，权重 type: Hero5/Eng3/Inf2/Sentry1/Outpost0.5）→ NAV=重心+偏哨兵侧 3m + 速度方向补偿 + 0.5m 迟滞 → 输出。

**关键事实**：
- `CostMap cm_` 只 load 不接线（pathLength/navPoint/at 从未调用）—— A* 避障未完成 🟡（2.3）
- cost_map.h 注释「28×15 像素 1px=1m」与实际 W=56/H=30（0.5m/px）**矛盾**；navPoint 输出为半米网格单位 ✨🟡
- `cfg["sentry"]` 段在**实际 main_config.yaml 中不存在**（README 示例有）—— 哨兵初始位置永远走 fallback（Red→(1.0,7.5)）✨🟢

### 3.6 judge_messager（judge_messager.cpp 216 行）

150ms 循环：location → 6 槽位（Hero/Eng/Inf3/Inf4/Inf5/Sentry，1→0,2→1,3→2,4→3,5→4,7→5）→ 打包 0x0305（每槽 x,y×100，mm 精度）→ 串口写入；同时**无条件发送 0x0301 双倍易伤 times=1**。

**关键事实**：
- `openSerial` 忽略 bps 参数，恒用 B115200（judge_messager.cpp:148-149）✨🟢（config 声明 115200 恰好一致，改配置会失效）
- 只写不读（无心跳/应答闭环）；双倍易伤无去抖 —— 对比 Python 版有 referee_receiver 完整接收闭环，C++ 版退化 🟡（2.4）
- 0x0301 的 sub_cmd 0x0121 / receiver 0x8080 / times=1 均为硬编码

### 3.7 registration（registration_node.cpp 789 行）

订阅 `lidar_pcds`（100Hz 缓存点云）→ 2s 定时器触发配准：

```
初始位姿获取（优先级）：config 注入(initial_pose.red/blue, yaw) → /initialpose 话题 → Quatro 全局配准
use_fixed 模式: Quatro bootstrap(3帧累积) → 之后 small_gicp(GICPFactor) 迭代
配准结果 → TF map→livox_frame（10ms 定时器转发）
```

**关键事实**：
- `use_quatro_`（纯 Quatro 每帧配准）、`!use_quatro_`（纯 small_gicp）两套分支
- 首次配准有跳变防护：config 初始位姿引导时，ICP 结果平移跳变 >8m 判定对称误匹配拒绝（registration_node.cpp:546-560）
- 连续失败 >1 次自动 reset 等待新初始位姿
- 🔴 **数据竞争**：pointcloud_sub_callback（订阅线程）写 source_cloud_ 与 timer_callback（2s 定时器）读 —— 无锁（MultiThreadedExecutor 下真实竞态）🟡（2.7）
- 启动参数默认 pcd_file=/home/rm/unit_test/ws/src/target.pcd —— 由 launch 从 main_config.yaml 注入覆盖

### 3.8 hik_camera_node（397 行）

海康 MVS 驱动：枚举（index/IP 选择）→ 3072×2048 BayerRG8(60fps) → 转换 BGR8 → 写共享内存（/dev/shm/hik_camera）→ 可选发布 image_raw。

**关键事实**：
- 共享内存布局：20B 头（w,h,frame_index,timestamp_ns）+ 4B 编码长度 + "bgr8" + 图像数据 —— 与 detect 的读端**双处硬编码，无公共头文件、无内存屏障/原子** 🟡（3.4）
- 写入顺序保证（先数据后帧号）是唯一同步机制；detect 端忙轮询 1ms
- shm 创建时 `shm_unlink` 清理残留 —— 好习惯

### 3.9 HomographyTransformer（71 行）

`pixelToField`：H_ground 透视 → map 像素 → 地图范围校验（±500px 容差）→ 掩码非黑点则切换 H_highland（高地层）→ mapToField 归一化到 28×15。

**关键事实**：
- 掩码判定 `pixel != Vec3b(0,0,0)`：任何非黑像素（含红色掩码绘制区）都触发高地变换 —— 依赖掩码图设计约定，需现场验证 ✨🟢
- `map_is_portrait` 分支（mapToField）公式需与标定脚本 cross-check
- config 路径 `perspective_calib.json` 与 lab 版 `perspective_calib_lab.json` 并存但**无场景切换机制**（detect 恒读前者）✨🟡

### 3.10 辅助资产

- **scripts/**（calibrate_perspective / calibrate_pcd_to_map / calibrate_3d_to_2d / cost_map_editor / lidar_test_gff）：标定工具链，Python，与 `main_config_lab.yaml`（新场景配置）配套 —— 处于 lab 场景适配进行中
- **bringup.sh**：rosbag 模式一键启动；硬编码 `LD_LIBRARY_PATH=/home/syh/TensorRT-10.13.2.6/lib`（部署耦合）
- **model/**: 活动模型在 `model/TensorRT/`（car.engine 28MB / armor2_hku.engine / digit_hku.engine）；`model/model/` 为遗留目录（旧引擎+onnx+trt_load.log），无配置引用 —— 待清理候选 ✨🟢
- **git 历史**：6 提交（e49c2df 首提 → ab242b5 nav decision）；工作区含未提交修改（lidar.cpp / main_config.yaml / registration_node.cpp）+ 大量未跟踪新文件（lab 场景 + 标定脚本 + judge_messager 新节点）

---

## 四、问题清单（合并 KNOWN_ISSUES + 本次新增）

> 状态：✅ 亲验 | 📋 源码推断。本次新增项标注 ✨。

### 4.1 高优先级（部署/功能风险）

| # | 问题 | 位置 | 状态 |
|---|------|------|------|
| 1.1 | radar_t.cpp 损坏副本（首行杂散 D，无法编译） | src/hnurm_radar/src/radar_t.cpp | ✅ **已于本次清理删除** |
| 1.2 | lidar 背景地图路径硬编码机器绝对路径，忽略 config | lidar.cpp:284 | ✅ |
| ✨1.9 | display_panel 地图路径拼接 `$HOME/rm_lidar_2027/...`，换机器即失效 | display_panel.cpp:34 | ✅ |
| 1.3 | location 双路冗余发布：pcdCb/otherCb 各自发布完整合并消息，实测 20Hz（≈9.4+10Hz）；消息内无重复，非功能故障，严重度中/低 | radar.cpp:599,707 | ✅ |
| 1.4 | DBSCAN 参数段 `lidar:` 在 detector_config.yaml 不存在，恒默认 0.15/7 | radar.cpp:255-256 | ✅ |
| 1.6 | detect 配置相对路径依赖 CWD；mask 路径硬编码不随 scene 切换 | detect.cpp:47,67 | ✅ |
| 1.7 | lidar 背景减除阈值 0.20 硬编码（README 写 0.15） | lidar.cpp:283 | ✅ |
| 1.8 | CUDA 调用零错误检查（infer 层静默 UB） | infer.cu 全篇 | ✅ |

### 4.2 中优先级（逻辑/健壮性）

| # | 问题 | 位置 | 状态 |
|---|------|------|------|
| 2.1 | tracker 的 SimpleKalmanFilter.predict 死代码 | tracker.cpp:42 | ✅ |
| 2.2 | display 找 "ATTACK"/"PATH" vs sentry 发 "DEF"/"ATK"，路径线永不绘制 | display_panel.cpp:181-187 | ✅ |
| 2.3 | CostMap A* 未接线 | sentry_decision.cpp:33 | ✅ |
| 2.4 | judge_messager 无接收闭环 + 双倍易伤无条件发送 | judge_messager.cpp:196-204 | ✅ |
| 2.5 | 丢弃模型颜色，BGR 差分重判 | detect.cpp:402-409 | ✅ |
| 2.6 | KF 噪声不可配（Q=50/R=0.1 硬编码） | radar.cpp:88-89 | ✅ |
| 2.7 | registration 数据竞争（无锁）+ 连续失败自动 reset | registration_node.cpp | ✅ |
| 2.8 | 无人机颜色缓存以指针为 key，vector 扩容失效 | radar.cpp:314,448 | ✅ |
| 2.9 | has_updated 复位时机失真 | radar.cpp:497 | ✅ |
| 2.10 | cameraMatch 时间窗 1s 偏宽 | radar.cpp:171 | ✅ |
| ✨2.13 | 1.3 复核修正：location 双路冗余发布（20Hz）为性能/整洁项，非功能故障；otherCb 发布的地面数据为未 predict 快照 | radar.cpp:599,707 | ✅ |
| ✨2.11 | MiniMap 绘制索引错位：`coords[i]`（仅有效装甲）与 `armors[i]`（全部）不一一对应，存在无效 armor 时坐标标签错配 | detect.cpp:485-495 | ✅ |
| ✨2.12 | Stage2 解析循环无 empty 检查：car_roi 为空时 scale3 除零 + 读取残留 host buffer | infer.cu:249-253 | ✅ |

### 4.3 低优先级（性能/整洁/防御）

| # | 问题 | 位置 | 状态 |
|---|------|------|------|
| 3.1 | Stage1 CPU resize + letterbox 纯拷贝 | infer.cu:189-190 | ✅ |
| 3.2 | Stage2 伪批量 + Stage3 逐板同步 | infer.cu:233-243,287 | ✅ |
| 3.3 | Stage3 scratch 显存 192² 实际 64²（9 倍浪费） | infer.cu:172-174 | ✅ |
| 3.4 | shm 布局双处硬编码，无公共头/原子 | hik 397 / detect 159 | ✅ |
| 3.5 | OpenCV GUI 依赖桌面环境 | display/detect | 📋 |
| 3.6 | decode_queue 无界积压 | detect.cpp:590 | ✅ |
| 3.7 | 魔法数字（场地边界/死区/label 映射）散落 | 多处 | ✅ |
| ✨3.8 | `detect_cache_` 只写不读死代码 | radar.cpp:296-301 | ✅ |
| ✨3.9 | radar rosbag 模式空回调订阅压缩图 | radar.cpp:272-276 | ✅ |
| ✨3.10 | TF 未就绪时 r2f_=Identity 照常发布伪坐标 | radar.cpp:390-392 | ✅ |
| ✨3.11 | `std_map_rel[0]` 空串 UB | display_panel.cpp:31 | ✅ |
| ✨3.12 | judge_messager 忽略 bps 参数恒 115200 | judge_messager.cpp:148 | ✅ |
| ✨3.13 | cost_map.h 注释与实现分辨率矛盾（28×15 vs 56×30） | cost_map.h:7-9 | ✅ |
| ✨3.14 | 标定文件无场景切换机制（perspective_calib vs _lab） | detect.cpp:66 | ✅ |
| ✨3.15 | `model/model/` 遗留目录（旧引擎/onnx/日志）无引用 | model/model/ | ✅ |
| ✨3.16 | bringup.sh 硬编码 TensorRT 路径 | bringup.sh:15 | ✅ |

### 4.4 复核修正（相对 KNOWN_ISSUES 的更新）

- KNOWN_ISSUES 1.1（radar_t.cpp）→ **已修复（删除）**，且 git 未跟踪，无历史负担
- KNOWN_ISSUES 3.8「label[0] 越界」→ 复核确认上游 `label.size()<2` 过滤（detect.cpp:375, radar.cpp:369），不触发；但 **2.11 的 coords/armors 错位才是真实显示 bug**
- KNOWN_ISSUES 2.7「doFirstRegistration_ 置真后位姿冻结」→ 复核：冻结不成立，置真后每次仍以 pre_result_ 为初值迭代 ICP（registration_node.cpp:497-508），位姿持续更新；但「2s 一次」的更新频率 + 无锁是真实问题
- KNOWN_ISSUES 1.2 补充：**display_panel.cpp:34 存在同类硬编码**（README 文档之外的新发现）
- **1.3 复核修正（2026-08-03 实测）**：「双份空中目标」不成立——单条 location 消息内每个目标仅一份；实测 20Hz = target_pointcloud(~9.4Hz) + livox/lidar_other(~10Hz) 两路各自发布完整合并消息所致，属冗余发送而非数据错误，严重度由高降为中/低（见 4.2 ✨2.13）

---

## 五、清理记录（本次执行）

| 项 | 操作 | 理由 |
|----|------|------|
| `src/hnurm_radar/src/radar_t.cpp` | 删除 | 首行损坏、未编译、git 未跟踪的死副本 |
| `src/hnurm_radar/src/a` | 删除 | 82 行 DBSCAN 类碎片（误重定向产物），未编译未跟踪 |
| `scripts/__pycache__/` | 删除 + .gitignore 补充 | Python 缓存污染 |
| `background0.pcd`（项目根） | 删除 | lidar_test_gff.py 的再生成输出物 |
| `map_2d_cropped.png`（项目根） | 删除 | 与 `source/maps/lab/` 完全重复（md5 一致） |
| `.gitignore` | 追加 `__pycache__/` `*.pyc` | 防复发 |

**保留但需注意**：git 状态中的 `D car_full.log / car_layers.json / car_precision.json / car_profile.json`（已删除的旧 profiling 输出）、`D hnurm_radr_node.cpp`（已被新架构取代）、未跟踪的 lab 场景文件与 judge_messager.cpp（新工作，待提交）。

---

## 六、改进路线建议（按收益排序）

### P0 — 部署前必须（比赛就绪）
1. **消除全部机器硬编码路径**（1.2 / 1.9 / 3.16）：config 统一 + `resolve_path()`（detector_config.yaml 注释声称有但从未实现）
2. **`debug_coordinate_publish` 设为 false 的发布前检查**（README 已强调，建议加启动断言）
3. ~~无人机双发布修复（1.3）~~ → 已复核降级：非功能故障，移至 P2（见 4.2 ✨2.13）

### P1 — 稳定性
4. CUDA 错误检查宏（1.8）
5. registration 配准加锁（2.7）
6. Stage2 空 ROI 防御（2.12）+ MiniMap 索引修复（2.11）
7. TF 就绪前拒绝发布（3.10）

### P2 — 数据质量
8. KF 噪声参数化（2.6）；DBSCAN 参数段补齐（1.4）
9. 颜色判定引入模型输出加权（2.5）
10. judge_messager 接收闭环（2.4，参照 Python 版 referee_receiver）
11. location 发布收敛单路（1.3 / ✨2.13）—— 双路 20Hz 冗余改为单路发布（otherCb 仅维护无人机 KF）

### P3 — 性能
11. Stage1 warp-affine 单 kernel 全 GPU（3.1）；Stage2/3 动态 batch（3.2）
12. Stage3 scratch 尺寸修正（3.3）

### P4 — 架构整洁
13. shm 公共布局头 + 原子帧号（3.4）
14. 场景化配置接线（sentry 段 / 标定文件切换）（3.14）
15. 删除 `model/model/` 遗留目录与死代码（3.8/3.15）

---

## 七、死代码/冗余代码清单（2026-08-03 grep 全量核查）

> 判定标准：全 src/ 范围内无调用者/无消费方。标注 ✂️ = 可安全删除（行为不变）。

### 7.1 完全死代码（无任何调用/消费）✂️

| # | 位置 | 说明 |
|---|------|------|
| D1 | tracker.cpp:42 `SimpleKalmanFilter::predict`（tracker.hpp:18） | 唯一 `KF.predict()` 在 radar.cpp:140，属 KalmanFilterPlus（cv::KalmanFilter），**非本类**；tracker 匹配纯 IoU |
| D2 | tracker.cpp:162 `stableLabel`（tracker.hpp:75） | 声明+定义，零调用 |
| D3 | tracker.cpp:234 `ObjectTracker::reset` | 零调用 |
| D4 | radar.cpp:296-301 `detect_cache_` + `detect_cache_mutex_` | 只写不读（emplace/pop 无消费），detectCb 已直接 cameraMatch |
| D5 | radar.cpp:272-276 `sub_comp_` | rosbag 模式空 lambda 订阅，无实际用途 |
| D6 | infer.cu:95,114 `preprocess_on_gpu` / `preprocess_classify_on_gpu`（非 _ex 版） | infer() 只用 _ex 版本；两函数+preprocess.cuh 声明均无调用者 |
| D7 | infer.hpp:47-48 `getLastCarTime`/`getLastArmorTime`；:44 `getLastTotalTime` | "兼容旧接口"，零调用 |
| D8 | sentry_decision.cpp:62 `zcx(Zone)` | 声明+定义，零调用 |
| D9 | sentry_decision.cpp:131-132 `atk_en`/`cen_en` | 计算后从未使用（is_def 只依赖 def_en/def_fr） |
| D10 | cost_map.h `pathLength`/`navPoint`/`at` 整套 A* | 仅 `cm_.load()` 被调用（sentry_decision.cpp:33），A* 全部未接线 |
| D11 | registration_node.cpp:169 `generate_initial_guesses` | 唯一调用被注释（:145） |
| D12 | hik_camera_node.cpp:156,382 `n_payload_size_` | 仅写入+日志，无消费 |
| D13 | display_panel.cpp:126-127 id 600-699 / 1600-1699 分支 | 无任何生产方（radar 只发 6/106） |
| D14 | display_panel.cpp:181-194 PATH/ATTACK 路径线 | 标签不匹配（sentry 发 DEF/ATK），永不触发，同 2.2 |
| D15 | detect_result EkfDiagnostics.msg / EkfDiagnosticsArray.msg | C++ 无生产方（Python 版遗留） |
| D16 | DetectResult.msg `xywh_box` | 无节点填充（detect 只填 xyxy_box） |
| D17 | registration 大段注释代码 | :145, :173-180, :420-426, :481-491, :509-533, :597-611 等（guesses_ 多猜测方案） |

### 7.2 死配置字段（声明但零消费）✂️

| 字段 | 文件 | 证据 |
|------|------|------|
| `stage_three_conf` | detector_config.yaml | src 内 0 引用 |
| `tracker_path`（botsort.yaml） | detector_config.yaml | 0 引用（detect 用 ObjectTracker） |
| `life_time` | detector_config.yaml | 0 引用 |
| `filter.process_noise / measurement_noise` | detector_config.yaml | 6/4 命中仅为 tracker 参数名，config 无消费 |
| `is_record / record_fps` | detector_config.yaml | 0 引用 |
| `lidar.height_threshold` | main_config.yaml | 0 引用 |
| `lidar.publish_projected` | main_config.yaml | 0 引用 |
| `lidar.voxel_size` | main_config.yaml | 0 引用（loadMap 硬编码 0.1f） |
| `lidar.background_map_path` | main_config.yaml | 0 引用（lidar.cpp:284 硬编码路径） |
| `camera.exposure_time / gain` | main_config.yaml | hik 节点用自身 declare_parameter 默认值，无 launch 注入 |
| `camera.video_source / test_image` | main_config.yaml | src 内 0 引用 |

### 7.3 冗余文件资产（无引用）✂️

| # | 路径 | 说明 |
|---|------|------|
| R1 | model/model/ 整个目录 | 旧引擎 + onnx + trt_load.log + best.engine.bak(0B)，配置无引用 |
| R2 | model/TensorRT/best.engine、armor2_nms.engine、armor_digit.engine | 旧版引擎，config 只引用 car/armor2_hku/digit_hku |
| R3 | model/ONNX/armor_yolo.onnx（44MB） | 无引用 |

### 7.4 半死/降级（需决策，勿直接删）

- **camera.mode "test"/"video"**：分支存在但无图像源生产（video_source/test_image 不消费），实际不可用；除非有外部 /image 发布者
- tracker 的 `label_stable`/`label` 机制：实际 label 恒为 "car"，相关计数逻辑冗余（删 stableLabel 后 label 字段仅供调试）

### 7.5 删除建议

7.1（D1-D17）+ 7.2 + 7.3 全部可安全删除，行为不变。建议：先提交当前未提交改动（lab 场景等）→ 再开一次 "dead code removal" 提交。7.4 需按产品决策。

---

## 附：文件清单（清理后）

```
RM_radar_Cpp_2027/
├── bringup.sh / README.md / .gitignore
├── configs/            # main_config(.yaml/_lab) detector converter(+_rosbag) perspective_calib(.json/_lab)
├── docs/               # KNOWN_ISSUES.md / PROJECT_ANALYSIS.md(本文件)
├── model/              # TensorRT/(3引擎) ONNX/ model/(遗留待删)
├── scripts/            # 5 个标定/测试脚本
├── source/             # maps/{competition_2026,lab} pointclouds/{background,registration}
├── src/
│   ├── hnurm_radar/    # 8 节点 + include/ (infer/tracker/cost_map/Homography/tensorrt/preprocess)
│   ├── registration/   # small_gicp + Quatro 配准
│   ├── detect_result/  # 5 自定义 msg
│   ├── livox_ros_driver2/ + Livox-SDK2/ + Quatro/
└── log/ build/ install/  # gitignored
```
