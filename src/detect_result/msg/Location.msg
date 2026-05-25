# Location.msg — 单个机器人的赛场坐标
#
# 由 camera_detector 节点在透视变换后发布，表示一个被检测到的机器人在赛场中的位置。
# 发布话题：/location（作为 Locations.locs 数组的元素）
# 订阅者：ekf_node（用于 EKF 滤波）
#
# 赛场坐标系：原点在赛场左下角（红方视角），x 轴向右 (0~28m)，y 轴向上 (0~15m)

float32 x        # 赛场 x 坐标 (m)，范围 0~28
float32 y        # 赛场 y 坐标 (m)，范围 0~15
float32 z        # 赛场 z 坐标 (m)，纯相机方案下通常为 0；激光雷达方案下为点云聚类中心高度
int32 id         # 车辆编号：红方 1~7，蓝方 101~107（对应 R1~R7 / B1~B7）
string label     # 颜色标签："Red" 或 "Blue"
