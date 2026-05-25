# EkfDiagnosticsArray.msg — 一帧中所有 EKF slot 的诊断指标集合
#
# 由 ekf_node 节点每 50ms 发布到 /ekf_diagnostics 话题。
# 包含 7 个 slot 的完整诊断数据，供 Foxglove Plot 可视化和
# display_panel 绘制协方差椭圆使用。

builtin_interfaces/Time stamp    # 发布时间戳
EkfDiagnostics[] slots           # 各 slot 的诊断数据（最多 7 个）
