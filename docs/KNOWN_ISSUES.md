# RM_radar_Cpp_2027 已知问题清单

> 记录日期：2026-08-01
> 范围：全项目源码精读 + 逐条复核（含模型/配置交叉验证）
> 可信度标注：✅ 亲验（直接读代码/模型验证）｜📋 源码分析结论

---

## 一、高优先级（功能/部署风险，建议尽快修复）

### 1.1 `radar_t.cpp` 是损坏的废弃副本 ✅
- 位置：`src/hnurm_radar/src/radar_t.cpp:1`
- 首行 ` D#include <rclcpp/rclcpp.hpp>` 有杂散字符 `D`，**无法编译**；与 `radar.cpp` 逐字节相同（diff 仅第 1 行），CMake 只编译 `radar.cpp`（CMakeLists.txt:131）。
- 影响：维护时极易误改/误编译；纯死代码。
- 建议：直接删除 `radar_t.cpp`。

### 1.2 lidar 背景地图路径硬编码为机器绝对路径 ✅
- 位置：`src/hnurm_radar/src/lidar.cpp:284`
- `map_path = "/home/syh/rm_lidar_2027/HNURM-radar-2026/data/pointclouds/background/RM2025.pcd"`——硬编码到**另一项目**的绝对路径，完全忽略 config `lidar.background_map_path`。
- 影响：换机器/换目录 100% 加载失败；与 HNURM-radar-2026 目录耦合。
- 建议：改读 `main_config.yaml` 的 `lidar.background_map_path`，并把背景 PCD 纳入本项目 `source/pointclouds/background/`。

### 1.3 location 双路冗余发布（实测 20Hz，非消息内重复） ✅
- 位置：`src/hnurm_radar/src/radar.cpp:599` 与 `:707`
- `pcdCb` 与 `otherCb` **各自**构建完整合并消息（地面 KF + last_known + `appendAirLocations()`）并各自发布 → 实测 `/location` ~20Hz（= `target_pointcloud` ~9.4Hz + `livox/lidar_other` ~10Hz 之和）。
- **复核修正（2026-08-03）**：单条 location 消息内无人机/地面目标各只有一份，**并非"双份目标"**；问题实质是合并消息被双倍频率冗余发送，且 `otherCb` 发布的地面部分是两次 `pcdCb` 之间未 predict 的快照。数据无错、消费端可容忍（sentry 0.5s 节流 / judge 150ms 覆盖 / display 直接重绘），严重度由"高"降为"中/低"。
- 建议：发布统一收敛到单一路径（如 `pcdCb` 侧），`otherCb` 仅维护无人机 KF 并仅在自身有更新时补充发布，避免整包重复。

### 1.4 DBSCAN 参数段配置不存在，恒用默认值 ✅
- 位置：`src/hnurm_radar/src/radar.cpp:255-256`
- 读取 `detector_config.yaml` 的 `lidar.cluster_eps / cluster_min_samples`，但该文件**没有 `lidar:` 段**（文件只有 path/params/filter/is_record），实际恒用默认 0.15m / 7 点。
- 影响：聚类参数无法调优；且与 `converter_config.yaml` 的 `cluster` 段（eps=0.40, min_points=10）不同源，容易混淆。
- 建议：在 `detector_config.yaml` 补 `lidar:` 段，或统一参数源。

### 1.5 配置声明但未消费（一批字段）✅
| 字段 | 位置（声明处） | 状态 |
|------|---------------|------|
| `stage_three_conf` | `detector_config.yaml` | 未消费（Stage3 数字分类无置信度过滤） |
| `path.tracker_path` (botsort.yaml) | `detector_config.yaml` | 未消费（detect 用 ObjectTracker） |
| `params.life_time` | `detector_config.yaml` | 未消费 |
| `filter.process_noise / measurement_noise` | `detector_config.yaml` | 未消费（SimpleKalmanFilter 用默认 0.01/0.1，tracker.hpp:16；radar 只消费 filter 段 3/5 字段） |
| `is_record / record_fps` | `detector_config.yaml` | 未消费 |
| `lidar.height_threshold` | `main_config.yaml` | 未消费 |
| `lidar.publish_projected` | `main_config.yaml` | 未消费 |
| `lidar.voxel_size` | `main_config.yaml` | 未消费（loadMap 硬编码 0.1f） |

### 1.6 detect 配置路径依赖 CWD + 场景硬编码 ✅
- 位置：`src/hnurm_radar/src/detect.cpp:47-48, 66-67`
- `YAML::LoadFile("configs/...")` 相对路径依赖工作目录（注释声称有 `resolve_path()` 但代码中没有调用）；`mask_img_path` 硬编码 `data/maps/competition_2026/pfa_map_mask_2025.jpg`，不随 `scene`（competition/lab）切换。
- 影响：从其他目录启动加载失败；切 lab 场景会用到错误的掩码/标定。
- 建议：统一路径解析；掩码路径进 `scenes` 配置。

### 1.7 lidar 背景减除阈值硬编码 ✅
- 位置：`src/hnurm_radar/src/lidar.cpp:283`
- `background_threshold_ = 0.20` 硬编码（与 config `lidar.background_threshold=0.2` 重复），改 config 不同步则失效；README 写的 0.15 与代码 0.20 不符。

### 1.8 CUDA 调用零错误检查（infer 层）✅
- 位置：`src/hnurm_radar/src/infer.cu` 全部 `cudaMalloc/cudaMemcpyAsync/cudaStreamSynchronize`
- 无任何返回值检查，失败即静默 UB（读垃圾数据）。建议至少包一层 `checkCuda` 宏。

---

## 二、中优先级（逻辑/健壮性问题）

### 2.1 tracker 卡尔曼为死代码 ✅
- 位置：`src/hnurm_radar/src/tracker.cpp:42`（`SimpleKalmanFilter::predict` 仅定义）
- detect.cpp 从未调用 `predict()`；匹配纯靠 IoU，KF 只被 `update()` 用（结果也没被消费，`last_rect` 直接用测量值）。
- 建议：接上预测（update 前先 predict 再 IoU）或删除 KF。

### 2.2 display 与 sentry 标签不匹配（路径线死代码）✅
- 位置：`src/hnurm_radar/src/display_panel.cpp:181,184` 找 `"ATTACK"/"PATH"`；`sentry_decision.cpp:169,174` 实际发 `"DEF"/"ATK"`（id=801/902）
- 影响：路径连线永不绘制。
- 建议：统一标签约定。

### 2.3 sentry 的 cost_map A* 未接线 ✅
- 位置：`src/hnurm_radar/src/sentry_decision.cpp:33` 仅 `cm_.load()`
- `pathLength / navPoint / at` 从未被调用——避障路径规划是未完成功能。

### 2.4 judge_messager 每 150ms 无条件发送双倍易伤 0x0301 ✅
- 位置：`src/hnurm_radar/src/judge_messager.cpp:196-204`
- `times=1` 恒定发送，无去抖/守卫；且 `judge_messager` **只写不读**（O_RDWR 但从不 read，无心跳/应答）。
- 对比：HNURM-radar-2026 Python 版有完整接收闭环（referee_receiver 子进程解析 0x020E 驱动双倍计数），C++ 版是退化版。

### 2.5 detect 丢弃模型颜色、BGR 差分重判 ✅
- 位置：`src/hnurm_radar/src/detect.cpp:402-409`
- Stage2 模型已输出颜色类别（0=dead/1=R/2=B），detect 只取 label 的数字部分（`substr(1)`），颜色改用 ROI 的 BGR 通道均值差分逐帧判定 + 7 帧多数投票。
- 影响：过曝/欠曝帧差分不稳，dead 装甲板无法区分；投票只能削弱不能消除。属设计选择，但模型输出被浪费。
- 建议：模型颜色为主、差分仅作校验，或加权融合。

### 2.6 KF 噪声参数不可配 ✅
- 位置：`src/hnurm_radar/src/radar.cpp:88-89, 98-100`
- `sigma_q_x/y=50, sigma_r_x/y=0.1` 硬编码，config 只覆盖 `delete_time / max_speed / detect_r`（:511-513）。`filter.process_noise` 等字段与 KF 的 Q/R 无关。
- 另外 `kf_delete_time_` 成员默认 **1.5f**（radar.cpp:287，非注释所写 0.2s），config 覆盖为 3.0 生效。

### 2.7 registration 数据竞争（无锁）✅
- 位置：`src/registration/src/registration_node.cpp`（`source_cloud_` 等成员）
- 订阅回调（executor 线程）与定时器回调（timer 线程）并发读写 `source_cloud_` / `source_cloud_downsampled_`，全程无 mutex（MultiThreadedExecutor 下是真实竞态）。
- 另：`doFirstRegistration_` 置真后不再复位（除非 reset），后续帧位姿冻结在首次结果。

### 2.8 无人机颜色迟滞缓存以指针为 key ✅
- 位置：`src/hnurm_radar/src/radar.cpp:314, 448`
- `air_color_cache_` 以 `KalmanFilterPlus*` 为 key，`KFs_air_` 是 vector，push_back 扩容后旧指针失效。低概率但易脆。

### 2.9 `has_updated` 复位时机使统计语义失真 ✅
- 位置：`src/hnurm_radar/src/radar.cpp:497（复位）vs :401-402（printStats 消费）`
- 复位发生在 Step1 predict 后而非"确实 update"后，纯视觉 KF 也会在一帧内显示雷达活跃。

### 2.10 cameraMatch 时间窗口 1s 偏宽 ✅
- 位置：`src/hnurm_radar/src/radar.cpp:171`（`TIME_THRESHOLD=1.0s`）
- 10Hz 场景下 1s 窗口可能误配到旧点，且为类内常量不随帧率自适应。

---

## 三、低优先级 / 改进建议

### 3.1 Stage1 CPU resize + 二次缩放（性能）✅
- `infer.cu:189-190`：整帧 `cv::resize` 到 1280×1280 在 CPU 上做（约 3-6ms），随后 letterbox kernel 实际是纯拷贝（scale=1）。
- 建议：仿 T-DT warp-affine 单 kernel 全 GPU 完成缩放+归一化。

### 3.2 Stage2 伪批量 + Stage3 逐装甲板同步（性能）✅
- `infer.cu:233-243`：Stage2 是"循环单张 enqueue"，非真 batch（引擎 batch=1）；`infer.cu:287`：Stage3 每块装甲板一次 `cudaStreamSynchronize`，流水线断流。
- 建议：Stage2/3 导出为动态 batch 引擎，一次 enqueue 推理全部 ROI。

### 3.3 Stage3 scratch 显存 9 倍浪费 ✅
- `infer.cu:172-174` 分配 192×192×3，实际输入 64×64（:284）。

### 3.4 共享内存头布局双处硬编码（hik_camera ↔ detect）✅
- `hik_camera_node.cpp:201-207`（writer）与 `detect.cpp:159,195-196`（reader）各自硬编码 20B 头部 + encoding 布局，无公共头文件、无内存屏障/atomic。
- 建议：抽公共 `shm_layout.h` + 原子帧序号。

### 3.5 display OpenCV GUI 依赖桌面环境 📋
- `display_panel.cpp` / detect 的 `cv::imshow` 在 headless 部署会崩；比赛机建议关 GUI 或用 /map_view 话题替代。

### 3.6 decode_queue 无界积压 ✅
- `detect.cpp:590`：rosbag/compressed 模式下 decode_queue_ 无大小上限（process/display 队列有 clear 保护）。

### 3.7 魔法数字散落 ✅
- 场地边界 -2/30/-2/17（radar.cpp:442,445,622）、无人机 z=2、色迟滞死区 13.5/14.5、label↔cid 两套映射（parseLabel vs labelToCarId）需人工保持一致。建议集中到 config。

### 3.8 与 T-DT 参考实现的差距（改进方向）
- T-DT 有：GPU decode+NMS kernel（运行时阈值可调）、真 batch 推理、Memory<T> 内存池 + checkRuntime、页锁定内存、intra-process 容器化。
- 本项目已用模型内置 EfficientNMS（正确选择，无需自写），但 conf/iou 阈值在模型导出时固化，现场调参需重导出。

---

## 四、复核修正记录（避免重复踩坑）

| 项 | 初判 | 复核结论 |
|----|------|---------|
| Stage1/2 NMS | "无 NMS，输出重叠框" | **误判**。模型内置 EfficientNMS（`car.engine`/`armor2_hku.engine` 含 `[NMS]_output` 层；Stage2 输出 `1×300×6`），NMS 由官方 plugin 在 GPU 完成，`infer.cu:208-211` 的 conf 过滤即配套消费 |
| tensorrt_inference.hpp 输出解析 | "不健壮" | 降级：对 EfficientNMS 的 `[1,N,6]` 布局是配套正确的，仅对非 NMS 布局不通用 |
| display label[0] 越界 | "UB" | 降级：上游保证 label ≥2 字符，实际不触发，属防御性问题 |
| kf_delete_time_ 默认 | "0.2s" | 修正为 1.5f（radar.cpp:287），config 覆盖 3.0 结论不变 |
| 1.3 "双份空中目标" | 消息内含重复目标 | **修正（2026-08-03 实测）**：单条消息内仅一份；实为两个回调各自发布完整合并消息导致 20Hz 双路冗余发布（=9.4Hz+10Hz），非功能故障，严重度降级 |

---

## 五、建议修复优先级排序

1. **删 `radar_t.cpp`**（1.1）—— 零风险，立即可做
2. **背景地图路径改读 config**（1.2）—— 部署必需
3. **location 发布收敛单路**（1.3）—— 非功能故障（实测 20Hz 冗余），性能/整洁项，可延后
4. **补 `detector_config.yaml` 的 `lidar:` 段**（1.4）
5. **统一路径解析 + 场景化掩码**（1.6）
6. **CUDA 错误检查宏**（1.8）
7. 其余按需排期（2.x 逻辑问题、3.x 性能优化）
