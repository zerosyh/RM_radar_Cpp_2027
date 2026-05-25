# detect_t.cpp - C++ 版本视觉检测器

这是基于 HNURM-radar-2026 中 `lidar_scheme/detector_node.py` 的 C++ 重写版本。

## 主要功能

- ROS2 C++ 节点实现
- 支持压缩图像订阅（rosbag 模式）
- 使用 OpenCV DNN 进行 YOLO 推理
- 简化的透视变换
- 多线程推理循环
- 实时图像显示和结果发布

## 依赖项

- ROS2 (rclcpp, sensor_msgs)
- OpenCV (包括 DNN 模块)
- yaml-cpp
- cv_bridge

## 使用方法

1. 确保模型文件存在：
   - `weights/stage_one.onnx`
   - `weights/stage_two.onnx`
   - `weights/stage_three.onnx`

2. 确保配置文件存在：
   - `configs/detector_config.yaml`
   - `configs/main_config.yaml`
   - `perspective_calib.json`

3. 运行节点：
   ```bash
   ros2 run hnurm_radar detect_t
   ```

## 注意事项

这是一个示例实现，包含以下简化：
- 使用 OpenCV DNN 而非 TensorRT
- 简化的 ByteTrack 实现（需要额外集成）
- 基础的透视变换（需要完整标定数据）
- 移除了录制和高级计时功能

要获得完整功能，需要：
- 集成 TensorRT 或其他推理引擎
- 实现完整的 ByteTrack 追踪
- 添加相机驱动支持
- 完善消息类型定义

## 架构

- `HomographyTransformer`: 透视变换类
- `Detector`: 主检测节点类
- 多线程设计：图像获取和推理分离