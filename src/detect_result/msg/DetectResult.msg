# DetectResult.msg — 单个目标的检测框信息
#
# 由 camera_detector 节点在 YOLO 三阶段推理 + ByteTrack 追踪后生成，
# 作为 Robots.detect_results 数组的元素发布到 /detect_view 话题。
# 主要用于可视化展示（display_panel），不用于坐标定位。
#
# 两种 bbox 格式说明：
#   xyxy_box: 绝对像素坐标，用于在原图上画框
#   xywh_box: 归一化坐标（相对于图像宽高），用于模型训练/评估

int32[4] xyxy_box      # 检测框的左上角和右下角像素坐标 [x1, y1, x2, y2]
float32[4] xywh_box    # 检测框的中心点+宽高归一化坐标 [cx, cy, w, h]，值域 0~1
int32 track_id         # ByteTrack 分配的跨帧追踪 ID，同一目标在连续帧中保持相同 ID
string label           # 三阶段分类结果标签，如 "R1"(红1号)、"B3"(蓝3号)、"NULL"(未识别)
float32 confidence      # 分类置信度（0~1）
# 单目3D赛场坐标（单位：米）
float64 field_x
float64 field_y
float64 field_z

