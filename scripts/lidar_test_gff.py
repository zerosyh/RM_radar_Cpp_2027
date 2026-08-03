#!/usr/bin/env python3
"""
实时订阅点云话题，累积多帧生成静态背景地图。
支持坐标变换（绕Y轴旋转，使 Z 轴垂直向上），并输出 PCD 文件。
"""

import argparse
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import threading
import os

class BackgroundCollector(Node):
    def __init__(self, args):
        super().__init__('background_collector')
        self.args = args
        self.lock = threading.Lock()
        self.all_points = []          # 存储所有帧的点
        self.frame_count = 0
        self.sub = self.create_subscription(
            PointCloud2, args.topic, self.callback, 10)
        self.get_logger().info(f'订阅话题: {args.topic}')
        self.get_logger().info(f'目标帧数: {args.max_frames} (0表示无限)')
        if args.apply_rotation:
            self.get_logger().info(f'应用旋转矩阵, 俯仰角: {args.lidar_pitch} rad')
        else:
            self.get_logger().info('不应用旋转')

    def apply_rotation(self, points):
        """应用绕Y轴的旋转矩阵，使 Z 轴垂直向上"""
        cos_p = np.cos(-self.args.lidar_pitch)
        sin_p = np.sin(-self.args.lidar_pitch)
        rot = np.array([
            [cos_p, 0, sin_p],
            [0,     1, 0],
            [-sin_p,0, cos_p]
        ], dtype=np.float32)
        return points @ rot.T

    def callback(self, msg):
        # 将点云消息转换为 numpy 数组
        points_list = []
        for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
            points_list.append([p[0], p[1], p[2]])
        if not points_list:
            return
        frame = np.array(points_list, dtype=np.float32)

        # 可选：应用旋转
        if self.args.apply_rotation:
            frame = self.apply_rotation(frame)

        # 安全添加到全局列表
        with self.lock:
            self.all_points.append(frame)
            self.frame_count += 1
            current = self.frame_count

        self.get_logger().info(f'已接收 {current} 帧，当前帧点数 {len(frame)}')

        # 如果达到最大帧数，则退出
        if self.args.max_frames > 0 and current >= self.args.max_frames:
            self.get_logger().info(f'已达到目标帧数 {self.args.max_frames}，准备保存...')
            rclpy.shutdown()

def main(args=None):
    parser = argparse.ArgumentParser(description='从实时话题生成背景点云地图')
    parser.add_argument('--topic', default='/livox/lidar', help='点云话题名称')
    parser.add_argument('--output', default='/home/syh/rm_lidar_2027/background0.pcd', help='输出 PCD 文件路径')
    parser.add_argument('--voxel_size', type=float, default=0.1, help='体素滤波大小（米），0表示不下采样')
    parser.add_argument('--max_frames', type=int, default=120, help='最大累积帧数（0表示无限，直到Ctrl+C）')
    parser.add_argument('--apply_rotation', action='store_true', help='是否应用旋转矩阵（水平校正）')
    parser.add_argument('--lidar_pitch', type=float, default=-0.1, help='雷达安装俯仰角（弧度），仅当--apply_rotation时有效')
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    collector = BackgroundCollector(parsed_args)

    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.get_logger().info('用户中断，准备保存...')
    finally:
        # 先销毁节点
        collector.destroy_node()
        # 尝试关闭 ROS，但忽略可能出现的异常（避免阻塞保存）
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Shutdown 过程中出现异常（已忽略）: {e}")

    # 合并所有帧的点云（确保保存逻辑执行）
    with collector.lock:
        if not collector.all_points:
            print('未收集到任何点云数据')
            return
        combined = np.vstack(collector.all_points)
        print(f'合并后总点数: {len(combined)}')

    # 创建 Open3D 点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(combined)

    # 体素滤波下采样
    if parsed_args.voxel_size > 0:
        pcd = pcd.voxel_down_sample(parsed_args.voxel_size)
        print(f'体素滤波后点数: {len(pcd.points)}')

    # 保存为 PCD 文件（使用绝对路径时确保目录存在）
    output_path = parsed_args.output
    if not os.path.isabs(output_path):
        output_path = os.path.join(os.getcwd(), output_path)
    try:
        o3d.io.write_point_cloud(output_path, pcd)
        print(f'背景地图已保存至: {output_path}')
    except Exception as e:
        print(f'保存文件失败: {e}')

if __name__ == '__main__':
    main()
