# RM_radar_Cpp_2027 Web 调试台 (radar_web_ui)

> 状态:2026-08-24 新增。定位/检测/点云数据已实时接入;解析波分析页面与接口已就绪,等待未来解析波模块。

## 1 快速开始

```bash
cd ~/rm_lidar_2027/RM_radar_Cpp_2027
colcon build --packages-select radar_web_ui
source install/setup.bash
ros2 run radar_web_ui web_ui_node --port 8766 --config configs/main_config.yaml
```

浏览器访问 `http://127.0.0.1:8766`。已在 `bringup.sh` 中增加该节点,回放/联调时会随系统一起启动。

三个页面:
- `① 赛场地图`:地图实时机器人 + 右侧详细数据与调试参数。
- `② 解析波分析`:目前显示“等待接入”;接口就绪,下面两种方式推送数据即可显示。
- `③ 运行状态`:话题速率、图像流、配置、EKF 诊断、日志、ROS graph。

## 2 解析波接入接口(供未来解析波项目对接)

### 2.1 方式 A:ROS2 话题(推荐,解析波为 C++ 节点时)

话题:`/wave_analysis`,类型:`std_msgs/msg/String`,内容为 UTF-8 JSON:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 在解析波节点中:
auto pub = create_publisher<std_msgs::msg::String>("wave_analysis", 10);
auto msg = std_msgs::msg::String();
msg.data = R"({
  "source": "wave_parser",
  "seq": 1,
  "status": "valid",
  "mode": "fmcw",
  "meta": {"sample_rate_hz": 100000.0, "center_frequency_hz": 1.0e10},
  "samples": [
    {"t_ms": 0.0,   "amplitude": 0.12, "snr_db": 18.0, "distance_m": 6.0, "angle_deg": 3.2},
    {"t_ms": 0.01,  "amplitude": 0.31, "snr_db": 17.5, "distance_m": 6.1, "angle_deg": 3.1}
  ]
})";
pub->publish(*msg);
```

### 2.2 方式 B:HTTP POST(独立程序/脚本/调试)

```bash
curl -X POST http://127.0.0.1:8766/api/wave/ingest \
  -H 'Content-Type: application/json' \
  -d '{
    "source":"wave_parser",
    "seq":1,
    "status":"valid",
    "mode":"fmcw",
    "meta":{"sample_rate_hz":100000},
    "samples":[
      {"t_ms":0.0,"amplitude":0.2,"snr_db":18.0,"distance_m":6.0,"angle_deg":3.2},
      {"t_ms":0.01,"amplitude":0.5,"snr_db":17.5,"distance_m":6.1,"angle_deg":3.1}
    ]
  }'
```

### 2.3 数据字段约定

| 字段 | 类型 | 必填 | 说明 |
| --- | --- | --- | --- |
| source | string | 否 | 来源节点名,默认取传输通道名 |
| seq | int | 否 | 帧序号 |
| status | string | 否 | `idle / running / valid / invalid / warn` |
| mode | string | 否 | 如 `fmcw / pulse / cw` |
| meta | object | 否 | 扩展信息;`sample_rate_hz` 用于 FFT 频率轴 |
| samples | array | **是** | 至少 1 点 |
| samples[].t_ms | number | 否 | 相对时间 ms |
| samples[].amplitude | number | 否 | 振幅(主绘图量) |
| samples[].phase_deg | number | 否 | 相位 ° |
| samples[].frequency_hz | number | 否 | 瞬时频率 Hz |
| samples[].snr_db | number | 否 | 信噪比 dB |
| samples[].distance_m | number | 否 | 距离 m(距离剖面) |
| samples[].angle_deg | number | 否 | 角度 °(角度剖面) |
| samples[].valid | bool | 否 | 默认 true |

完整 Schema:`GET /api/wave/schema`。

## 3 地图页数据来源与坐标

- 地图底图:`main_config.yaml → scenes.<scene>.std_map`(competition 为 `source/maps/competition_2026/std_map.png`)。
- 坐标系与 `display_panel` 一致:裁判系统系,红方补给站原点,X 朝蓝方 0~28,Y 0~15;画布 y 向下,故做 y 翻转。
- 红色 id 1~7,蓝色 id 101~107;id 6/106 或 z>0.8 按空中目标绘制。
- 负 id 为 radar.cpp 的 `last_known_locs_`,绘制为灰叉。
- `sentry_targets` 绘制紫点,`ai_nav` 绘制橙色菱形。

## 4 调试参数

页面右侧调试参数可实时调整,保存到 `~/.config/rm_radar_web_ui/debug_params.json`(仅影响 Web UI 显示/缓存,不修改 C++ 算法参数):
- 地图底图/网格/标注开关、轨迹开关与点数、标记半径、超时阈值、解析波最大采样点数等。

## 5 运行状态页

- 话题速率由后端 1s 滑窗统计;点云话题只统计点数不缓存点云。
- `map_view`/`detect_view` 以 MJPEG 直连 ROS Image 回调,延迟即 ROS 回调延迟。
- `ekf_diagnostics` 订阅类型已预留(`detect_result/msg/EkfDiagnosticsArray`),当前 C++ 项目未发布,显示占位说明。
- ROS graph 每 5s 刷新一次。

## 6 模拟解析波(页面验收用)

项目当前没有真实解析波,可启动模拟节点验证第 2 页:

```bash
ros2 run radar_web_ui wave_mock_node --mode fmcw --period 0.2 --count 512
```

`--mode` 支持 `fmcw / pulse / cw`。

---

## 附录:零基础学习入口

如果你想从零学会自己写这种调试 Web UI,请阅读同目录下的
[`WEBUI_LEARNING_GUIDE.md`](WEBUI_LEARNING_GUIDE.md)。该文档以本包为完整案例,从 HTML/CSS/JS/Python 最小知识开始,逐模块分析后端与前端实现,并给出 10 步从零实现最小版调试 UI 的练习路线。
