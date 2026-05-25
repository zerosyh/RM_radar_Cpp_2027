#!/bin/bash
# bringup_rosbag.sh — Rosbag 回放模式一键启动脚本（含视觉检测）
#
# 前置条件：
#   1. configs/main_config.yaml 中 camera.mode 设为 "rosbag"
#   2. configs/main_config.yaml 中 camera.rosbag_path 填写 rosbag 目录路径
#   3. configs/detector_config.yaml 模型路径配置正确
#   4. colcon build 已完成
#   5. 已安装 gnome-terminal

# 移除MVS库路径以避免libusb冲突
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|/opt/MVS/lib/64:||g')

# 添加 TensorRT 库路径（根据实际安装位置调整）
export LD_LIBRARY_PATH=/home/syh/TensorRT-10.13.2.6/lib:$LD_LIBRARY_PATH

source /opt/ros/humble/setup.bash
source install/setup.bash

# 工作目录切换到项目根目录（确保 detect 节点读取配置的相对路径正确）
cd ~/rm_lidar_2027/RM_radar_Cpp_2027

cmds=(
    "ros2 launch livox_ros_driver2 rviz_HAP_launch.py"
    "ros2 run hnurm_radar lidar"
    "ros2 launch registration registration.launch.py"
    
    "ros2 run hnurm_radar detect"             # 新增：视觉检测节点
    "ros2 bag play /home/syh/下载/全明星赛第一局 --rate 5.0"
)

for cmd in "${cmds[@]}"
do
    echo "Current CMD : $cmd"
    gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; $cmd; exec bash;"
    sleep 0.1
done
