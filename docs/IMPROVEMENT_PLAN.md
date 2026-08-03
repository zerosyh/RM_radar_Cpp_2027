# detect 与裁判系统通信 完善方案

> 制定日期：2026-08-03
> 依据：RM_radar_Cpp_2027 源码精读 + T-DT_Radar 参考实现 + 《RoboMaster 2026 通信协议 V2.0.0（20260626）》85 页逐节核对 + HNURM Python 版（referee_receiver.py / judge_messager.py）对照
> 状态：待实施（P0 建议赛前必做，P1 性能，P2 增强）

---

## 一、裁判系统通信完善（judge_messager）

### 1.1 协议核对结论（V2.0.0 vs 当前 C++ 实现）

| 项 | 协议要求 | 当前实现 | 判定 |
|----|---------|---------|------|
| 帧格式 | SOF 0xA5 + data_length(2) + seq(1) + CRC8 + cmd_id(2) + data + CRC16(整包) | buildFrame 一致 | ✅ |
| 波特率 | 电源管理模块←→机器人 115200 | B115200 | ✅ |
| 雷达 ID | 红 9 / 蓝 109 | my_id_ = 9/109 | ✅ |
| 0x8080 | 裁判系统服务器（哨兵/雷达自主决策） | receiver = 0x8080 | ✅ |
| **0x0305 雷达数据** | **48B：对方6车 + 己方6车，槽位 [英雄/工程/3步/4步/空中6/哨兵]，cm，全 0 = 未发送** | 24B 仅 6 槽 [Hero/Eng/3步/4步/**5步**/哨兵]，无对方/己方区分 | ❌ 长度+槽位双错 |
| **0x0301+0x0121 双倍易伤** | 内容段 **8B**：radar_cmd(1B，**单调递增每次+1**) + password_cmd(1B) + 密钥 6B(ASCII)；触发前提：0x020E 显示拥有机会 | 内容段 2B（times=1 恒发 150ms），无递增、无机会判断、无密钥 | ❌ 全部不符 |
| 0x020E 接收 | bit0-1 触发机会 / bit2 对方正在触发 / bit3-4 加密等级 / bit5 可改密钥 | 无接收 | ❌ 缺失 |
| 0x020C 接收 | 2B：对方易伤标记进度(≥100)、己方特殊标识(≥50)、激光瞄准位 | 无接收 | ❌ 缺失 |
| 0x0A01-0x0A06 | 雷达无线链路（电磁波）：对方位置/血量/发弹/金币/增益/密钥 | 无接收 | ❌ 缺失（依赖硬件） |
| 自定义客户端 | Protobuf v3 + MQTT，192.168.12.1:3333 | 无 | ⏳ 后续 |

> 注：HNURM Python 版同样存在 0x0305 槽位（5步而非空中）与 0x0121 旧结构问题——两版均需按 2026 协议修正。

### 1.2 实施内容

#### P0-1：串口接收闭环（Receiver 线程）
- 参照 `HNURM-radar-2026/.../referee_receiver.py` 移植为 C++：读线程 + 环形缓冲 + find_sof 滑动搜索 + CRC8/CRC16 校验 + `parse_cmd_id` 分派
- 解析 0x020E（双倍易伤机会/对方触发中/加密等级/可改密钥）→ 原子变量/互斥共享给发送线程
- 解析 0x020C（标记进度）→ 可驱动"被标记≥100 的车"决策（哨兵辅助）

#### P0-2：0x0121 双倍易伤正确化
- 内容段 8B：`radar_cmd`（uint8，单调递增，每触发一次 +1，**非 2B times**）+ `password_cmd`（1=更新己方密钥 / 2=验证对方密钥）+ 6B ASCII 密钥
- 触发状态机：收到 0x020E 机会>0 且 未在触发中 → 发 radar_cmd+1；已触发则等待生效结束（0x020E bit2）
- 发送频率：0x0301 上行上限 30Hz（当前 150ms 周期满足，但需去重——无状态变化不重发）
- 密钥策略（可配置开关）：byte1=1 开局/加密等级提升时轮换己方密钥；收到 0x0A06 对方密钥后 byte1=2 验证

#### P0-3：0x0305 修正为 48B
- 12 槽位：对方 [英雄1/工程2/3步3/4步4/空中6/哨兵7] + 己方 [同序]，每槽 (x,y) uint16 cm
- 槽位映射修正：5 号步兵槽位 → **空中 6 号**（0x0305 无 5 号步兵位置）
- 己方坐标数据源：location 里己方哨兵(7/107)直接填；己方其他车在 `debug_coordinate_publish=false` 时不发布 → 填 0（视为未发送）或由 radar 另路提供全量（方案内决策：默认 0 + 哨兵）
- 边界：坐标超边界显示在边缘（协议明示），0/0 视为未发送——沿用现有 clamp

#### P1-4：鲁棒性
- `openSerial` 使用 config 的 bps（当前恒 115200 忽略参数）
- 串口写加锁（Python 版有 serial_lock）
- 掉线重连/打开失败告警（当前失败仅 ERROR 后继续空转）

#### P2-5：雷达无线链路 + 客户端协议（硬件/需求确认后）
- 0x0A01-0x0A06 需雷达接收模块（电磁波）；确认硬件再实现
- 自定义客户端（MQTT+Protobuf）：选手端小地图 / RadarInfoToClient —— 需求确认后另立模块

---

## 二、detect 完善（借鉴 T-DT）

### 2.1 T-DT 参考实现要点（已读源码）

| 项 | T-DT 实现 | 我们现状 |
|----|----------|---------|
| 批量推理 | `armor_yolo->forwards(images)` 一次 enqueue 全部车 ROI（引擎 minBatch 1/optBatch 5/maxBatch 12）；classifier forwards 批量装甲（optBatch 10） | Stage2 循环单张 enqueue；Stage3 每装甲一次同步 |
| GPU decode+NMS | decode_kernel_v5/v8 + fast_nms_kernel，conf/iou **运行时可调**（yolo::load(path, Type, conf, iou)） | 模型内嵌 EfficientNMS，阈值导出时固化 |
| CUDA 错误检查 | `checkRuntime` / `checkKernel` 宏 | 零检查（KNOWN_ISSUES 1.8） |
| 预处理 | AffineMatrix::compute warp-affine + 单 kernel GPU 缩放归一化 | CPU cv::resize + letterbox kernel 纯拷贝（3.1） |
| 内存 | BaseMemory：cudaMallocHost 页锁定 + gpu/unified 池 + reference | cudaMalloc 裸指针 + scratch 复用（显存浪费 3.3） |
| 引擎自举 | engine 缺失自动跑 onnx2trt.py（minBatch/optBatch/maxBatch） | 缺失即异常退出 |
| 颜色判定 | BGR 通道差分（与我们的方案同源，车级单次判定） | 差分 + 7 帧投票（增强版，保留） |

### 2.2 实施内容

#### P1-1：Stage2/3 真 batch 推理
- 引擎导出动态 batch（参照 T-DT onnx2trt.py 的 minBatch/optBatch/maxBatch），`forwards(images)` 一次 enqueue
- 消除 Stage3 逐装甲板 `cudaStreamSynchronize`（流水线断流）

#### P1-2：checkRuntime 宏
- infer.cu / tensorrt_inference.hpp 所有 cuda 调用包一层宏（T-DT NvidiaInterface.cu:12 模板）

#### P1-3：Stage1 GPU 预处理
- 仿 AffineMatrix::compute：GPU 单 kernel 完成 resize+letterbox+归一化，去掉 CPU cv::resize（Stage1 预处理 1ms→≈0）

#### P2-4：运行时 conf/iou（可选，工作量大）
- 自研 decode+NMS kernel 替代模型内嵌 EfficientNMS，conf/iou 参数化 → 现场调参无需重导出引擎

#### P2-5：内存池
- Stage3 scratch 尺寸修正（192²→64²，省 9 倍显存）；页锁定内存 + 预分配 host 缓冲

#### P2-6：健壮性
- tensorrt_inference.hpp 输出解析：1D 输出（nbDims==1）时 outDim 未初始化 bug
- Stage2 空 ROI 防御（infer.cu:249 除零 + 残留数据）

---

## 三、优先级与依赖

| 优先级 | 项 | 依赖 | 工作量 |
|--------|----|------|--------|
| **P0** | 1.2-1（接收闭环） | 无 | 中（移植 467 行 Python） |
| **P0** | 1.2-2（0x0121 正确化） | P0-1 | 小 |
| **P0** | 1.2-3（0x0305 48B） | 无 | 小 |
| P1 | 2.2-1（真 batch） | 模型重导出 | 中（引擎+代码） |
| P1 | 2.2-2（checkRuntime） | 无 | 小 |
| P1 | 2.2-3（GPU 预处理） | 无 | 中 |
| P2 | 2.2-4/5/6、1.2-4/5 | 硬件/需求确认 | — |

> 建议顺序：P0 三项先做（赛前合规必需）→ P1 性能三项 → P2 按需。模型重导出需要原始训练 onnx（car_nms/armor2_hku/classify 已保留）。
