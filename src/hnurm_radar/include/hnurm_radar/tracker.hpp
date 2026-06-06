#ifndef TRACKER_HPP_
#define TRACKER_HPP_

#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include <deque>
#include <unordered_map>
#include <memory>

// ============================================================
// 简单卡尔曼滤波器 (x, y + vx, vy)
// ============================================================
class SimpleKalmanFilter {
public:
    SimpleKalmanFilter(float process_noise = 0.01f, float measurement_noise = 0.1f);
    void init(float x, float y);
    std::pair<float, float> predict(float dt);
    std::pair<float, float> update(float mx, float my);
    bool initialized() const { return init_; }

private:
    bool init_ = false;
    float P_[4][4], x_[4];
    float F_[4][4], H_[2][4], Q_[4][4], R_[2][2];
};

// ============================================================
// 单个跟踪目标
// ============================================================
struct TrackedObject {
    int track_id = -1;
    std::string label;              // 最终标签 e.g. "R1", "B2", "NULL"
    cv::Rect last_rect;             // 原图检测框
    cv::Point2f bottom_center;      // 底部中心（透视Transform用）
    int hits = 0;
    int lost = 0;
    int label_stable = 0;           // 连续相同标签帧数
    std::unique_ptr<SimpleKalmanFilter> kf;

    TrackedObject() = default;
    TrackedObject(const TrackedObject& other)
        : track_id(other.track_id), label(other.label),
          last_rect(other.last_rect), bottom_center(other.bottom_center),
          hits(other.hits), lost(other.lost), label_stable(other.label_stable) {
        if (other.kf) {
            kf = std::make_unique<SimpleKalmanFilter>();
            kf->init(other.bottom_center.x, other.bottom_center.y);
        }
    }
    TrackedObject& operator=(const TrackedObject& other) {
        if (this != &other) {
            track_id = other.track_id;
            label = other.label;
            last_rect = other.last_rect;
            bottom_center = other.bottom_center;
            hits = other.hits;
            lost = other.lost;
            label_stable = other.label_stable;
            if (other.kf) {
                kf = std::make_unique<SimpleKalmanFilter>();
                kf->init(other.bottom_center.x, other.bottom_center.y);
            }
        }
        return *this;
    }
    TrackedObject(TrackedObject&&) = default;
    TrackedObject& operator=(TrackedObject&&) = default;

    TrackedObject(int id, const cv::Rect& r, const std::string& lbl,
                  const cv::Point2f& bottom);
    float iou(const cv::Rect& other) const;
    void update(const cv::Rect& r, const std::string& lbl, const cv::Point2f& bottom);
    void markLost();
    std::string stableLabel(int min_frames = 10) const;
};

// ============================================================
// 帧间追踪器（IoU 贪心匹配）
// ============================================================
class ObjectTracker {
public:
    ObjectTracker(float iou_thr = 0.3f, int max_lost = 30, int min_hits = 3);
    std::vector<TrackedObject> update(const std::vector<cv::Rect>& rects,
                                      const std::vector<std::string>& labels,
                                      const std::vector<cv::Point2f>& bottoms);
    void reset();
    const std::unordered_map<int, TrackedObject>& tracks() const { return tracks_; }

private:
    float iou_thr_;
    int max_lost_, min_hits_;
    int next_id_ = 0;
    std::unordered_map<int, TrackedObject> tracks_;
};

#endif // TRACKER_HPP_
