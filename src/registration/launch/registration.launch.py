"""
registration.launch.py — ICP 点云配准启动文件

功能：
  启动 registration_node 和 rviz2，进行点云地图配准并提供 TF 变换。

  支持两种模式：
    - 实时模式：使用场景配置中的 PCD 文件
    - rosbag 模式：当 camera.mode == "rosbag" 且 rosbag_pcd_file 存在时，
      自动使用 rosbag_pcd_file 作为点云地图

使用方式：
  ros2 launch registration registration.launch.py
"""

import os
import sys

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _get_project_root():
    """通过 ament 包安装路径向上推导 colcon workspace 根目录"""
    share_dir = get_package_share_directory('registration')
    return os.path.normpath(
        os.path.join(share_dir, '..', '..', '..', '..')
    )


def _load_main_config(project_root):
    """从 main_config.yaml 读取完整配置并返回 dict。

    读取失败时返回空 dict。
    """
    try:
        from ruamel.yaml import YAML
        ruamel = YAML()
        config_path = os.path.join(project_root, "configs", "main_config.yaml")
        with open(config_path, encoding="utf-8") as f:
            return ruamel.load(f) or {}
    except Exception as e:
        print(f"[registration.launch] Failed to load main_config.yaml: {e}",
              file=sys.stderr)
        return {}


def _load_scene_pcd_paths(project_root, cfg=None):
    """从 main_config.yaml 读取当前场景的 PCD 路径。

    返回 (pcd_file, downsampled_pcd_file) 的绝对路径元组。
    读取失败时回退到默认的 data/pointclouds/registration/pcds*.pcd。
    """
    pcd_rel = "data/pointclouds/registration/pcds.pcd"
    down_rel = "data/pointclouds/registration/pcds_downsampled.pcd"
    try:
        if cfg is None:
            cfg = _load_main_config(project_root)
        scene_name = cfg.get("global", {}).get("scene", "competition")
        scenes = cfg.get("scenes", {})
        scene_cfg = scenes.get(scene_name, {})
        if scene_cfg:
            pcd_rel = scene_cfg.get("pcd_file", pcd_rel)
            down_rel = scene_cfg.get("downsampled_pcd", down_rel)
        print(f"[registration.launch] scene={scene_name}, "
              f"pcd={pcd_rel}, downsampled={down_rel}")
    except Exception as e:
        print(f"[registration.launch] Failed to load scene config: {e}, using defaults")

    return (
        os.path.join(project_root, pcd_rel),
        os.path.join(project_root, down_rel),
    )


def _load_rosbag_pcd_override(project_root):
    """检查 rosbag 模式，如果配置了 rosbag_pcd_file 则返回其路径。

    返回 (pcd_file, downsampled_pcd_file) 或 None（非 rosbag 模式 / 未配置）。
    """
    try:
        from ruamel.yaml import YAML
        ruamel = YAML()
        config_path = os.path.join(project_root, "configs", "main_config.yaml")
        with open(config_path, encoding="utf-8") as f:
            cfg = ruamel.load(f)

        camera_cfg = cfg.get("camera", {})
        mode = camera_cfg.get("mode", "")
        rosbag_pcd_file = camera_cfg.get("rosbag_pcd_file", "")

        # 如果是相对路径，拼接 project_root
        if rosbag_pcd_file and not os.path.isabs(rosbag_pcd_file):
            rosbag_pcd_file = os.path.join(project_root, rosbag_pcd_file)

        if mode == "rosbag" and rosbag_pcd_file and os.path.isfile(rosbag_pcd_file):
            print(f"[registration.launch] rosbag 模式: 使用 rosbag_pcd_file = "
                  f"{rosbag_pcd_file}", file=sys.stderr)
            # downsampled 版不存在时使用同一文件（registration 节点会自动降采样）
            return rosbag_pcd_file, rosbag_pcd_file
        elif mode == "rosbag" and rosbag_pcd_file:
            print(f"[registration.launch] ⚠ rosbag_pcd_file 不存在: "
                  f"{rosbag_pcd_file}，回退到场景 PCD", file=sys.stderr)
    except Exception as e:
        print(f"[registration.launch] 读取 rosbag 配置失败: {e}", file=sys.stderr)

    return None


def _load_initial_pose(cfg):
    """根据 my_color 从 main_config.yaml 选择对应的初始位姿。

    返回 (x, y, z, yaw) 元组，读取失败时返回 (0, 0, 0, 0)。
    """
    try:
        my_color = cfg.get("global", {}).get("my_color", "Blue")
        color_key = my_color.lower()  # "red" or "blue"
        initial_pose_cfg = cfg.get("initial_pose", {})
        pose = initial_pose_cfg.get(color_key, {})
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        z = float(pose.get("z", 0.0))
        yaw = float(pose.get("yaw", 0.0))
        print(f"[registration.launch] my_color={my_color}, "
              f"initial_pose: x={x}, y={y}, z={z}, yaw={yaw}",
              file=sys.stderr)
        return x, y, z, yaw
    except Exception as e:
        print(f"[registration.launch] 读取 initial_pose 失败: {e}, "
              f"使用默认值 (0,0,0,0)", file=sys.stderr)
        return 0.0, 0.0, 0.0, 0.0


def generate_launch_description():
    project_root = _get_project_root()
    reg_share = get_package_share_directory('registration')
    rviz_config = os.path.join(project_root, 'configs', 'icp.rviz')

    # 读取 main_config.yaml（一次读取，多处使用）
    main_cfg = _load_main_config(project_root)

    # 确定 PCD 路径：rosbag 模式优先使用 rosbag_pcd_file
    rosbag_override = _load_rosbag_pcd_override(project_root)
    if rosbag_override:
        pcd_file, downsampled_pcd_file = rosbag_override
    else:
        pcd_file, downsampled_pcd_file = _load_scene_pcd_paths(project_root, main_cfg)

    # 根据 my_color 选择初始位姿
    ip_x, ip_y, ip_z, ip_yaw = _load_initial_pose(main_cfg)

    # 读取 default.yaml 并合并 PCD 路径覆盖
    # 注意：ROS2 launch 中 params_file（含 /**:）的参数优先级高于 dict 参数，
    # 因此不能简单 [params_file, {override}] 叠加，必须先读取 YAML 再合并。
    default_params_path = os.path.join(reg_share, 'params', 'default.yaml')

    reg_params = {}
    try:
        with open(default_params_path, encoding='utf-8') as f:
            raw = yaml.safe_load(f)
        # default.yaml 格式: {"/**": {"ros__parameters": {...}}}
        if raw and '/**' in raw:
            reg_params = dict(raw['/**'].get('ros__parameters', {}))
        elif raw:
            reg_params = dict(raw)
    except Exception as e:
        print(f'[registration.launch] 读取参数文件失败: {e}，使用默认参数',
              file=sys.stderr)

    # 覆盖 PCD 路径
    reg_params['pcd_file'] = pcd_file
    reg_params['downsampled_pcd_file'] = downsampled_pcd_file
    print(f'[registration.launch] 最终 PCD: pcd_file={pcd_file}, '
          f'downsampled_pcd_file={downsampled_pcd_file}', file=sys.stderr)

    # 注入初始位姿参数
    reg_params['initial_pose_x'] = ip_x
    reg_params['initial_pose_y'] = ip_y
    reg_params['initial_pose_z'] = ip_z
    reg_params['initial_pose_yaw'] = ip_yaw

    actions = [
        Node(
            package='registration',
            executable='registration_node',
            output='screen',
            parameters=[reg_params]
        ),
    ]

    # RViz2（点云可视化）
    if os.path.isfile(rviz_config):
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config]
            )
        )

    return LaunchDescription(actions)
