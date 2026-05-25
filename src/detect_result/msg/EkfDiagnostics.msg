# EkfDiagnostics.msg — 单个 EKF slot 的诊断指标
#
# 由 ekf_node 每 50ms 发布到 /ekf_diagnostics（作为 EkfDiagnosticsArray.slots 元素）。
# 用于实时评估检测质量、EKF 滤波效果、目标跟踪连续性等。
#
# slot 映射：
#   0-4 → 地面机器人 (ID 1-5 / 101-105)
#   5   → 哨兵 (ID 7 / 107)
#   6   → 空中机器人 (ID 6 / 106)

int32 slot_id                    # EKF 滤波器编号 0-6
int32 robot_id                   # 实际机器人 ID (1-7 / 101-107)，未激活时为 0
float32 detection_rate_hz        # 滑动窗口内的检测频率 (Hz)
float32 time_since_last_det_ms   # 距上次收到该 slot 原始检测的毫秒数
float32 jitter                   # 帧间位移抖动: |当前滤波位置 − 上次滤波位置| (m)
float32 innovation_x             # EKF 新息 X 分量: 观测值 − 预测值 (m)
float32 innovation_y             # EKF 新息 Y 分量: 观测值 − 预测值 (m)
float32 innovation_norm          # 新息范数: sqrt(ix² + iy²) (m)
float32 raw_vs_filtered_dist     # |原始检测坐标 − EKF 滤波坐标| (m)
float32 covariance_trace         # 协方差矩阵迹: tr(P)，反映 EKF 整体不确定性
float32 cov_xx                   # P[0,0] X 方向位置方差 (m²)，用于绘制协方差椭圆
float32 cov_yy                   # P[2,2] Y 方向位置方差 (m²)，用于绘制协方差椭圆
