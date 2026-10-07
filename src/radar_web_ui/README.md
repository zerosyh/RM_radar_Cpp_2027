# radar_web_ui — RM_radar_Cpp_2027 调试 Web UI

基于 ROS2 Humble 的调试可视化节点(纯 Python 标准库 HTTP 服务),为雷达站提供:

1. **赛场地图页**:28×15 m 裁判系统坐标系底图,实时绘制 `location` 机器人位置、运动轨迹、最后已知位置、`sentry_targets` 哨兵决策与 `ai_nav` AI 导航点;右侧显示机器人详细数据与可调调试参数。
2. **解析波分析页**:时域波形 / FFT 频谱 / 距离剖面 / 角度剖面。当前项目尚未实现解析波,**页面与接口已就绪**,未来解析波模块接入即可显示。
3. **运行状态页**:各 ROS 话题速率/数据年龄、map_view 与 detect_view 图像流、main_config 配置摘要、EKF 诊断占位、节点事件日志、ROS Graph。

## 构建

```bash
cd ~/rm_lidar_2027/RM_radar_Cpp_2027
colcon build --packages-select radar_web_ui
source install/setup.bash
```

## 运行

```bash
ros2 run radar_web_ui web_ui_node --port 8766 --config configs/main_config.yaml
# 或
ros2 launch radar_web_ui web_ui.launch.py
```

浏览器打开 `http://<上位机IP>:8766`(本机为 `http://127.0.0.1:8766`)。

ROS2 参数:`host`(默认 0.0.0.0)、`port`(默认 8766)、`config_path`(默认 configs/main_config.yaml)。

## 订阅话题

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| location | detect_result/msg/Locations | 雷达融合定位(地图主数据源) |
| ekf_location_filtered | detect_result/msg/Locations | EKF 滤波定位(预留) |
| sentry_targets | detect_result/msg/Locations | 哨兵决策标记 |
| ai_nav | detect_result/msg/Locations | AI 模型导航点 |
| detect_result | detect_result/msg/Robots | 视觉检测框(投影/详情) |
| map_view | sensor_msgs/msg/Image | display_panel 小地图 MJPEG |
| detect_view | sensor_msgs/msg/Image | 检测画面 MJPEG |
| target_pointcloud / livox/lidar_other | sensor_msgs/msg/PointCloud2 | 点云流量统计 |
| ekf_diagnostics | detect_result/msg/EkfDiagnosticsArray | EKF 诊断(未来发布者) |
| **wave_analysis** | **std_msgs/msg/String** | **解析波 JSON 接入** |

## HTTP 接口

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/` | Web UI |
| GET | `/api/state` | 全量状态 JSON |
| GET | `/api/events?generation=N` | SSE 实时推送 |
| GET | `/api/wave` | 解析波当前数据/历史 |
| GET | `/api/wave/schema` | 解析波 JSON Schema |
| POST | `/api/wave/ingest` | 解析波数据接入(联调) |
| POST | `/api/wave/clear` | 清空解析波数据 |
| GET/PUT | `/api/debug` | 调试参数读写 |
| GET | `/api/map.png` | 当前场景地图底图 |
| GET | `/stream/map_view` | map_view MJPEG |
| GET | `/stream/detect_view` | detect_view MJPEG |

解析波 JSON 示例见 `docs/WEB_UI.md`。

## 零基础学习

配套教学文档:`docs/WEBUI_LEARNING_GUIDE.md`(项目根目录 docs 下),以本包为案例讲解 Web UI 的完整写法。
