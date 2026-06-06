#include "hnurm_radar/tracker.hpp"
#include <cstring>
#include <algorithm>
#include <cmath>
#include <limits>

// ============================================================
// SimpleKalmanFilter
// ============================================================
SimpleKalmanFilter::SimpleKalmanFilter(float process_noise, float measurement_noise) {
    std::memset(F_, 0, sizeof(F_));
    F_[0][0] = 1; F_[0][2] = 1;
    F_[1][1] = 1; F_[1][3] = 1;
    F_[2][2] = 1;
    F_[3][3] = 1;

    std::memset(H_, 0, sizeof(H_));
    H_[0][0] = 1;
    H_[1][1] = 1;

    std::memset(Q_, 0, sizeof(Q_));
    Q_[0][0] = process_noise;
    Q_[1][1] = process_noise;
    Q_[2][2] = process_noise;
    Q_[3][3] = process_noise;

    std::memset(R_, 0, sizeof(R_));
    R_[0][0] = measurement_noise;
    R_[1][1] = measurement_noise;

    std::memset(x_, 0, sizeof(x_));
    std::memset(P_, 0, sizeof(P_));
    P_[0][0] = P_[1][1] = P_[2][2] = P_[3][3] = 1;
}

void SimpleKalmanFilter::init(float x, float y) {
    x_[0] = x; x_[1] = y;
    x_[2] = 0; x_[3] = 0;
    init_ = true;
}

std::pair<float, float> SimpleKalmanFilter::predict(float dt) {
    if (!init_) return {0,0};
    F_[0][2] = dt; F_[1][3] = dt;
    float nx[4] = {};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            nx[i] += F_[i][j] * x_[j];
    std::memcpy(x_, nx, sizeof(x_));

    // P = F * P * F^T + Q
    float FP[4][4] = {{0}};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                FP[i][j] += F_[i][k] * P_[k][j];
    float FPFt[4][4] = {{0}};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                FPFt[i][j] += FP[i][k] * F_[j][k];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            P_[i][j] = FPFt[i][j] + Q_[i][j];
    return {x_[0], x_[1]};
}

std::pair<float, float> SimpleKalmanFilter::update(float mx, float my) {
    if (!init_) return {0,0};
    float y[2] = {mx - x_[0], my - x_[1]};

    // S = H * P * H^T + R
    float HP[2][4] = {{0}};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                HP[i][j] += H_[i][k] * P_[k][j];
    float S[2][2] = {{0}};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 4; ++k)
                S[i][j] += HP[i][k] * H_[j][k];
    S[0][0] += R_[0][0]; S[1][1] += R_[1][1];

    float d = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (std::abs(d) < 1e-8f) return {x_[0], x_[1]};

    float invS[2][2] = {{S[1][1]/d, -S[0][1]/d},
                        {-S[1][0]/d, S[0][0]/d}};

    // K = P * H^T * S^{-1}
    float PHt[4][2] = {{0}};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 4; ++k)
                PHt[i][j] += P_[i][k] * H_[j][k];
    float K[4][2] = {{0}};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
                K[i][j] += PHt[i][k] * invS[k][j];

    for (int i = 0; i < 4; ++i)
        x_[i] += K[i][0] * y[0] + K[i][1] * y[1];

    // P = (I - K*H) * P
    float IKH[4][4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            IKH[i][j] = (i==j ? 1.0f : 0.0f);
            for (int k = 0; k < 2; ++k)
                IKH[i][j] -= K[i][k] * H_[k][j];
        }
    }
    float newP[4][4] = {{0}};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                newP[i][j] += IKH[i][k] * P_[k][j];
    std::memcpy(P_, newP, sizeof(P_));

    return {x_[0], x_[1]};
}

// ============================================================
// TrackedObject
// ============================================================
TrackedObject::TrackedObject(int id, const cv::Rect& r, const std::string& lbl,
                             const cv::Point2f& bottom)
    : track_id(id), last_rect(r), label(lbl), bottom_center(bottom),
      hits(1), lost(0), label_stable(1) {
    kf = std::make_unique<SimpleKalmanFilter>();
    kf->init(bottom.x, bottom.y);
}

float TrackedObject::iou(const cv::Rect& other) const {
    cv::Rect inter = last_rect & other;
    float ia = (float)inter.area();
    if (ia <= 0.0f) return 0.0f;
    return ia / (float)(last_rect.area() + other.area() - ia);
}

void TrackedObject::update(const cv::Rect& r, const std::string& lbl,
                           const cv::Point2f& bottom) {
    last_rect = r;
    bottom_center = bottom;
    hits++;
    lost = 0;
    if (lbl == label || lbl.empty())
        label_stable++;
    else {
        label_stable = 0;
        label = lbl;
    }
    if (kf) kf->update(bottom.x, bottom.y);
}

void TrackedObject::markLost() {
    lost++;
}

std::string TrackedObject::stableLabel(int min_frames) const {
    if (label_stable >= min_frames && !label.empty() && label != "NULL")
        return label;
    return "NULL";
}

// ============================================================
// ObjectTracker
// ============================================================
ObjectTracker::ObjectTracker(float iou_thr, int max_lost, int min_hits)
    : iou_thr_(iou_thr), max_lost_(max_lost), min_hits_(min_hits) {}

std::vector<TrackedObject> ObjectTracker::update(
    const std::vector<cv::Rect>& rects,
    const std::vector<std::string>& labels,
    const std::vector<cv::Point2f>& bottoms) {

    // 收集活跃 trajectory
    std::vector<int> alive_ids;
    for (auto& [id, tr] : tracks_)
        if (tr.lost < max_lost_) alive_ids.push_back(id);

    std::vector<TrackedObject> out;
    std::vector<bool> tr_match(alive_ids.size(), false);
    std::vector<bool> det_match(rects.size(), false);

    // 贪心 IoU 匹配
    for (int iter = 0; iter < (int)std::min(alive_ids.size(), rects.size()); ++iter) {
        float best_iou = iou_thr_;
        int best_t = -1, best_d = -1;
        for (size_t i = 0; i < alive_ids.size(); ++i) {
            if (tr_match[i]) continue;
            for (size_t j = 0; j < rects.size(); ++j) {
                if (det_match[j]) continue;
                float iou_val = tracks_[alive_ids[i]].iou(rects[j]);
                if (iou_val > best_iou) { best_iou = iou_val; best_t = (int)i; best_d = (int)j; }
            }
        }
        if (best_t >= 0) {
            tr_match[best_t] = true;
            det_match[best_d] = true;
            auto& tr = tracks_[alive_ids[best_t]];
            tr.update(rects[best_d], labels[best_d], bottoms[best_d]);
            if (tr.hits >= min_hits_) out.push_back(tr);
        } else break;
    }

    // 未匹配 track → lost
    for (size_t i = 0; i < alive_ids.size(); ++i) {
        if (!tr_match[i]) {
            auto& tr = tracks_[alive_ids[i]];
            tr.markLost();
            if (tr.hits >= min_hits_) out.push_back(tr);
        }
    }

    // 未匹配检测 → 新 track
    for (size_t j = 0; j < rects.size(); ++j) {
        if (!det_match[j]) {
            int id = next_id_++;
            tracks_.emplace(id, TrackedObject(id, rects[j], labels[j], bottoms[j]));
        }
    }

    // 清理 lost 过久的 track
    for (auto it = tracks_.begin(); it != tracks_.end(); ) {
        if (it->second.lost >= max_lost_) it = tracks_.erase(it);
        else ++it;
    }
    return out;
}

void ObjectTracker::reset() {
    tracks_.clear();
    next_id_ = 0;
}
