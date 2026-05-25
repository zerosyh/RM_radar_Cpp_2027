# Locations.msg — 一帧中所有检测到的机器人位置集合
#
# 由 camera_detector 节点每帧发布到 /location 话题，
# 也由 ekf_node 滤波后发布到 /ekf_location_filtered 话题。
# 包含当前帧中所有被成功识别并定位的机器人坐标。
#
# 典型使用：
#   ros2 topic echo /location                  # 查看原始定位
#   ros2 topic echo /ekf_location_filtered     # 查看 EKF 滤波后的定位

Location[] locs  # Location 数组，每个元素代表一个机器人的赛场坐标
