# Robots.msg — 一帧中所有目标的检测结果集合
#
# 由 camera_detector 节点每帧发布到 /detect_view 话题，
# 包含当前帧中所有被 YOLO 检测到并经 ByteTrack 追踪的目标信息。
# 主要供 display_panel 节点订阅，用于在画面上绘制检测框和标签。
#
# 与 Locations.msg 的区别：
#   Robots   → 图像域信息（检测框像素坐标、追踪 ID、分类标签），用于可视化
#   Locations → 赛场域信息（透视变换后的赛场坐标），用于定位和裁判系统通信

DetectResult[] detect_results  # DetectResult 数组，每个元素代表一个检测到的目标
