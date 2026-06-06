#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <detect_result/msg/robots.hpp>
#include <detect_result/msg/detect_result.hpp>
#include <detect_result/msg/location.hpp>
#include <detect_result/msg/locations.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <chrono>
#include <cmath>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <mutex>

using namespace std::chrono_literals;

// ==================== DBSCAN 聚类 ====================
class DBSCANCluster {
public:
    DBSCANCluster(float eps, int min_samples) : eps_(eps), min_samples_(min_samples) {}
    std::vector<std::vector<Eigen::Vector3f>> cluster(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
        if (cloud.empty()) return {};
        int n = cloud.size();
        std::vector<int> labels(n, -1);
        int cid = 0;
        for (int i = 0; i < n; ++i) {
            if (labels[i] != -1) continue;
            auto nb = regionQuery(cloud, i);
            if ((int)nb.size() < min_samples_) { labels[i] = -2; continue; }
            labels[i] = cid;
            std::vector<int> seed(nb.begin(), nb.end());
            for (size_t j = 0; j < seed.size(); ++j) {
                int q = seed[j];
                if (labels[q] == -2) labels[q] = cid;
                if (labels[q] != -1) continue;
                labels[q] = cid;
                auto qn = regionQuery(cloud, q);
                if ((int)qn.size() >= min_samples_)
                    for (int nb2 : qn) if (labels[nb2] == -1 || labels[nb2] == -2) seed.push_back(nb2);
            }
            cid++;
        }
        std::vector<std::vector<Eigen::Vector3f>> clusters(cid);
        for (int i = 0; i < n; ++i)
            if (labels[i] >= 0) clusters[labels[i]].emplace_back(cloud[i].x, cloud[i].y, cloud[i].z);
        return clusters;
    }
private:
    float eps_;
    int min_samples_;
    std::vector<int> regionQuery(const pcl::PointCloud<pcl::PointXYZ>& cloud, int idx) {
        std::vector<int> nb;
        const auto& pt = cloud[idx];
        for (size_t i = 0; i < cloud.size(); ++i) {
            float dx = cloud[i].x - pt.x, dy = cloud[i].y - pt.y, dz = cloud[i].z - pt.z;
            if (std::sqrt(dx*dx+dy*dy+dz*dz) <= eps_) nb.push_back(i);
        }
        return nb;
    }
};

// ==================== T-DT Kalman_filter_plus 完全对齐 ====================
struct KalmanFilterPlus {
    cv::KalmanFilter KF;
    float            last_time = 0;           // T-DT: 累加未更新时间
    std::chrono::steady_clock::time_point timer; // T-DT: 计算 dt
    float            delete_time = 2.0;       // T-DT: 超时阈值
    pcl::PointXY     predict_point;           // T-DT: 预测点
    std::deque<std::pair<int,int>> detect_history; // T-DT: (color, number)
    static constexpr int MAX_HISTORY = 20;    // T-DT: max_history = 20
    float  detect_r = 1;                      // T-DT: detect_r
    float  car_max_speed = 2.5;               // T-DT: car_max_speed
    float  dt_ = 0.1f;                        // T-DT: dt_
    float  sigma_q_x = 50.0f, sigma_q_y = 50.0f;  // T-DT: Q
    float  sigma_r_x = 0.1f, sigma_r_y = 0.1f;    // T-DT: R
    bool   has_updated = false;               // T-DT: has_updated
    int    hits = 1;                          // 累计更新次数（用于确认稳定航迹）

    // T-DT: Kalman_filter_plus(pcl::PointXY& input, rclcpp::Time time)
    KalmanFilterPlus(const pcl::PointXY& input, float)
        : last_time(0), delete_time(2.0f), detect_r(1.0f), car_max_speed(2.5f)
        , dt_(0.1f), sigma_q_x(50.0f), sigma_q_y(50.0f), sigma_r_x(0.1f), sigma_r_y(0.1f)
    {
        predict_point = input;
        timer = std::chrono::steady_clock::now();
        KF.init(4, 2, 0, CV_32F);
        cv::Mat state(4, 1, CV_32F);
        state.at<float>(0) = input.x;
        state.at<float>(1) = 0.0f;
        state.at<float>(2) = input.y;
        state.at<float>(3) = 0.0f;
        KF.statePost = state;
        // T-DT: transitionMatrix = [[1,dt_,0,0],[0,1,0,0],[0,0,1,dt_],[0,0,0,1]]
        KF.transitionMatrix = (cv::Mat_<float>(4,4) <<
            1, dt_, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, dt_,
            0, 0, 0, 1);
        // T-DT: measurementMatrix = [[1,0,0,0],[0,0,1,0]]
        KF.measurementMatrix = (cv::Mat_<float>(2,4) <<
            1, 0, 0, 0,
            0, 0, 1, 0);
        // T-DT: processNoiseCov = Q
        KF.processNoiseCov = (cv::Mat_<float>(4,4) <<
            sigma_q_x * dt_*dt_*dt_ / 3, sigma_q_x * dt_*dt_ / 2, 0, 0,
            sigma_q_x * dt_*dt_ / 2, sigma_q_x * dt_, 0, 0,
            0, 0, sigma_q_y * dt_*dt_*dt_ / 3, sigma_q_y * dt_*dt_ / 2,
            0, 0, sigma_q_y * dt_*dt_ / 2, sigma_q_y * dt_);
        // T-DT: measurementNoiseCov = [[r,0],[0,r]]
        KF.measurementNoiseCov = (cv::Mat_<float>(2,2) << sigma_r_x, 0, 0, sigma_r_y);
        // T-DT: errorCovPost = I
        cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));
        has_updated = true;
    }

    // T-DT: float get_time()
    float get_time() {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - timer);
        return duration.count() / 1000.0;
    }

    // T-DT: void update_predict_point()
    //    dt_ = get_time();
    //    timer = std::chrono::steady_clock::now();
    //    auto result = KF.predict();
    //    last_time += dt_;
    //    predict_point = result
    void update_predict_point() {
        dt_ = get_time();
        timer = std::chrono::steady_clock::now();
        KF.transitionMatrix = (cv::Mat_<float>(4,4) <<
            1, dt_, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, dt_,
            0, 0, 0, 1);
        // 更新 Q（dt_ 变了）
        KF.processNoiseCov = (cv::Mat_<float>(4,4) <<
            sigma_q_x * dt_*dt_*dt_ / 3, sigma_q_x * dt_*dt_ / 2, 0, 0,
            sigma_q_x * dt_*dt_ / 2, sigma_q_x * dt_, 0, 0,
            0, 0, sigma_q_y * dt_*dt_*dt_ / 3, sigma_q_y * dt_*dt_ / 2,
            0, 0, sigma_q_y * dt_*dt_ / 2, sigma_q_y * dt_);
        auto result = KF.predict();
        last_time += dt_;
        predict_point.x = result.at<float>(0);
        predict_point.y = result.at<float>(2);
    }

    // T-DT: bool match(pcl::PointXY& input)
    //    Distance(predict_point, input) < car_max_speed * dt_ + detect_r
    bool match(const pcl::PointXY& input) {
        float dx = predict_point.x - input.x;
        float dy = predict_point.y - input.y;
        float dist = std::sqrt(dx*dx + dy*dy);
        return dist < car_max_speed * dt_ + detect_r;
    }

    // T-DT: void update(pcl::PointXY& input, rclcpp::Time time)
    //    KF.correct(meas);
    //    predict_point = statePost
    //    has_updated = true;
    //    last_time = 0;  // ← 关键！update 重置 last_time
    void update(const pcl::PointXY& input) {
        cv::Mat meas(2, 1, CV_32F);
        meas.at<float>(0) = input.x;
        meas.at<float>(1) = input.y;
        KF.correct(meas);
        predict_point.x = KF.statePost.at<float>(0);
        predict_point.y = KF.statePost.at<float>(2);
        has_updated = true;
        last_time = 0;  // T-DT: update 时 last_time 归零
        hits++;
    }

    // T-DT: void camera_match(rclcpp::Time& time, pcl::PointXY& input, int color, int number)
    //    找 history 中时间最近的点，距离 < detect_r 则注入
    //    简化版：直接用 predict_point 匹配（和原版略有不同，但效果等价）
    void cameraMatch(float field_x, float field_y, int color, int number) {
        float dx = predict_point.x - field_x;
        float dy = predict_point.y - field_y;
        if (std::sqrt(dx*dx + dy*dy) < detect_r) {
            detect_history.push_back({color, number});
            if (detect_history.size() > MAX_HISTORY)
                detect_history.pop_front();
        }
    }

    // T-DT: int get_color()
    int getColor() const {
        if (detect_history.empty()) return 1;
        int red = 0, blue = 0;
        for (auto& [c, n] : detect_history) {
            if (c == 0) blue++;
            else if (c == 2) red++;
            // c==1 (unknown) 不投票
        }
        if (red > blue) return 2;
        if (blue > red) return 0;
        // 平票：取最近一票
        auto& [last_c, last_n] = detect_history.back();
        return (last_c == 2) ? 2 : 0;
    }

    // T-DT: int get_number()
    int getNumber() const {
        int color = getColor();
        std::map<int, int> num_map;
        for (auto& [c, n] : detect_history) {
            if (c == color) num_map[n]++;
        }
        int best_num = 0, best_cnt = 0;
        for (auto& [n, cnt] : num_map) {
            if (cnt > best_cnt) { best_cnt = cnt; best_num = n; }
        }
        return best_num;
    }
};

// ==================== 主体 Radar 节点 ====================
// 完全对齐 T-DT kalman_filter.cpp
class RadarNode : public rclcpp::Node {
public:
    RadarNode() : Node("RadarNode") {
        RCLCPP_INFO(get_logger(), "RadarNode 启动...");
        auto main_cfg = YAML::LoadFile("configs/main_config.yaml");
        auto det_cfg = YAML::LoadFile("configs/detector_config.yaml");
        std::string mode = main_cfg["camera"]["mode"].as<std::string>("hik");
        my_color_ = main_cfg["global"]["my_color"].as<std::string>("Red");
        debug_publish_all_ = main_cfg["global"]["debug_coordinate_publish"].as<bool>(false);
        labels_ = det_cfg["params"]["labels"].as<std::vector<std::string>>();
        if (mode == "rosbag") {
            lidar_frame_ = main_cfg["camera"]["tf_source_frame"].as<std::string>("livox_frame");
        } else {
            lidar_frame_ = "livox";
        }

        auto converter_file = (mode == "rosbag")
            ? "configs/converter_config_rosbag.yaml"
            : "configs/converter_config.yaml";
        auto converter_cfg = YAML::LoadFile(converter_file);
        auto ext = converter_cfg["calib"]["extrinsic"];
        auto Rd = ext["R"]["data"].as<std::vector<double>>();
        auto Td = ext["T"]["data"].as<std::vector<double>>();
        int rr = ext["R"]["rows"].as<int>(), rc = ext["R"]["cols"].as<int>();
        Eigen::Matrix3d Rm;
        for (int i = 0; i < rr && i < 3; ++i)
            for (int j = 0; j < rc && j < 3; ++j) Rm(i,j) = Rd[i*rc+j];
        Eigen::Vector3d tv(Td[0], Td[1], Td[2]);
        ext_mat_.setIdentity();
        ext_mat_.topLeftCorner<3,3>() = Rm;
        ext_mat_.topRightCorner<3,1>() = tv;
        ext_inv_ = ext_mat_.inverse();

        auto fc = det_cfg["filter"];
        kf_delete_time_ = fc["max_inactive_time"].as<float>(3.0f);
        kf_max_speed_   = fc["max_velocity"].as<float>(2.5f);
        kf_detect_r_    = fc["jump_threshold"].as<float>(1.0f);
        float db_eps = det_cfg["lidar"]["cluster_eps"].as<float>(0.15f);
        int db_min   = det_cfg["lidar"]["cluster_min_samples"].as<int>(7);
        cluster_ = std::make_unique<DBSCANCluster>(db_eps, db_min);

        tf_buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_lis_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);
        rclcpp::QoS qos(rclcpp::KeepLast(3), rmw_qos_profile_sensor_data);
        rclcpp::QoS qr(rclcpp::KeepLast(10)); qr.reliable();

        sub_pcd_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "target_pointcloud", qr, std::bind(&RadarNode::pcdCb, this, std::placeholders::_1));
        sub_det_ = create_subscription<detect_result::msg::Robots>(
            "detect_result", qos, std::bind(&RadarNode::detectCb, this, std::placeholders::_1));
        sub_other_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox/lidar_other", qr, std::bind(&RadarNode::otherCb, this, std::placeholders::_1));
        cluster_air_ = std::make_unique<DBSCANCluster>(kf_air_dbscan_eps_, kf_air_dbscan_min_);
        pub_loc_ = create_publisher<detect_result::msg::Locations>("location", qos);
        if (mode == "rosbag") {
            auto t = main_cfg["camera"]["compressed_image_topic"].as<std::string>("/compressed_image");
            sub_comp_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                t, qos, [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) { (void)msg; });
        }
        tmr_tf_ = create_wall_timer(1s, std::bind(&RadarNode::onTf, this));
        tmr_stats_ = create_wall_timer(5s, std::bind(&RadarNode::printStats, this));
        RCLCPP_INFO(get_logger(), "RadarNode OK. color=%s labels=%zu", my_color_.c_str(), labels_.size());
    }

private:
    std::string my_color_, lidar_frame_ = "livox_frame";
    bool debug_publish_all_;
    std::vector<std::string> labels_;
    Eigen::Matrix4d ext_mat_, ext_inv_, r2f_, r2f_inv_;
    float kf_delete_time_ = 2.0f;
    float kf_max_speed_   = 2.5f;
    float kf_detect_r_    = 1.0f;

    std::vector<KalmanFilterPlus> KFs_;
    std::mutex kf_mutex_;  // 保护 KFs_，detectCb 和 pcdCb 可能并发访问

    // 视觉缓存：时间戳对齐用
    struct DetectFrame {
        std::chrono::steady_clock::time_point stamp;
        detect_result::msg::Robots::SharedPtr msg;
    };
    std::deque<DetectFrame> detect_cache_;
    std::mutex detect_cache_mutex_;  // T-DT: vector<Kalman_filter_plus> KFs
    std::unique_ptr<DBSCANCluster> cluster_;

    // ---- 无人机检测 ----
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_other_;
    std::unique_ptr<DBSCANCluster> cluster_air_;
    std::vector<KalmanFilterPlus> KFs_air_;
    std::mutex kf_air_mutex_;
    static constexpr float kf_air_max_speed_  = 10.0f;
    static constexpr float kf_air_delete_time_ = 3.0f;
    static constexpr float kf_air_detect_r_    = 1.5f;
    static constexpr float kf_air_dbscan_eps_  = 0.3f;
    static constexpr int   kf_air_dbscan_min_  = 5;
    std::unordered_map<KalmanFilterPlus*, char> air_color_cache_;  // per-KF 迟滞颜色

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd_;
    rclcpp::Subscription<detect_result::msg::Robots>::SharedPtr sub_det_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_comp_;
    rclcpp::Publisher<detect_result::msg::Locations>::SharedPtr pub_loc_;
    rclcpp::TimerBase::SharedPtr tmr_tf_;
    std::shared_ptr<tf2_ros::Buffer> tf_buf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_lis_;

    // 统计
    rclcpp::TimerBase::SharedPtr tmr_stats_;
    int stat_pcd_frames_ = 0;
    int stat_detect_frames_ = 0;

    std::pair<int,int> parseLabel(const std::string& lab) const {
        if (lab.size() < 2) return {1, -1};
        char prefix = lab[0];
        std::string num_part = lab.substr(1);
        int number = -1;
        for (int i = 0; i < (int)labels_.size(); ++i) {
            if (labels_[i] == num_part) { number = i; break; }
        }
        int color = 1;
        if (prefix == 'B') color = 0;
        else if (prefix == 'R') color = 2;
        return {color, number};
    }

    int labelToCarId(const std::string& lab) const {
        if (lab.size() < 2) return 0;
        char prefix = lab[0];
        std::string num_part = lab.substr(1);
        int num = 0;
        if (num_part == "S" || num_part == "s") num = 7;
        else if (num_part == "Q" || num_part == "q") num = 5;
        else { try { num = std::stoi(num_part); } catch(...) { return 0; } }
        if (num < 1 || num > 7) return 0;
        // 红方1-7 蓝101-107（与 Python 版一致）
        if (prefix == 'R') {
            return num;          // 红车 1-7
        } else if (prefix == 'B') {
            return 100 + num;    // 蓝车 101-107
        } else if (prefix == 'D') {
            return (my_color_ == "Red") ? 100 + num : num;
        }
        return 0;
    }

    // T-DT: detect_callback — 对每个 KF 做 camera_match
    void detectCb(const detect_result::msg::Robots::SharedPtr msg) {
        stat_detect_frames_++;
        // 立即用 detect 结果对每个 KF 做 cameraMatch
        std::lock_guard<std::mutex> lock(kf_mutex_);
        for (auto& d : msg->detect_results) {
            if (d.label.size() < 2) continue;
            auto [color, number] = parseLabel(d.label);
            if (color == 1 || number < 0) continue;
            for (auto& kf : KFs_) {
                kf.cameraMatch(d.field_x, d.field_y, color, number);
            }
        }
        // 同时缓存最近 detect 结果，给 pcdCb 做时间对齐参考
        {
            std::lock_guard<std::mutex> lock(detect_cache_mutex_);
            detect_cache_.emplace_back(DetectFrame{std::chrono::steady_clock::now(), msg});
            if (detect_cache_.size() > 5) detect_cache_.pop_front();
        }
    }

    void onTf() {
        try {
            auto tf = tf_buf_->lookupTransform("map", lidar_frame_,
                tf2::TimePointZero, tf2::durationFromSec(0.1));
            r2f_ = tf2::transformToEigen(tf).matrix();
            r2f_inv_ = r2f_.inverse();
        } catch(...) {
            r2f_.setIdentity();
            r2f_inv_.setIdentity();
        }
    }

    void printStats() {
        int vis_cnt = 0, lidar_cnt = 0, fusion_cnt = 0, published_cnt = 0;
        std::lock_guard<std::mutex> lock(kf_mutex_);
        for (auto& kf : KFs_) {
            if (!kf.detect_history.empty()) vis_cnt++;
            if (kf.has_updated) lidar_cnt++;
            if (!kf.detect_history.empty() && kf.has_updated) fusion_cnt++;
            // 与发布逻辑完全一致
            if (!kf.detect_history.empty()) {
                int color  = kf.getColor();
                int number = kf.getNumber();
                if (color == 0 || color == 2) {
                    if (number >= 0 && number < (int)labels_.size()) {
                        std::string label = (color == 0 ? "B" : "R") + labels_[number];
                        int cid = labelToCarId(label);
                        if (cid != 0) {
                            bool enemy = false, my7 = false;
                            if (my_color_ == "Red") {
                                enemy = (cid >= 100);
                                my7   = (cid == 7);
                            } else {
                                enemy = (cid < 100 && cid > 0);
                                my7   = (cid == 107);
                            }
                            if (debug_publish_all_ || enemy || my7) published_cnt++;
                        }
                    }
                }
            }
        }
        RCLCPP_INFO(get_logger(),
            "[轨迹统计] KF:%zu  视觉:%d  点云:%d  融合:%d  发布:%d  pcd帧:%d  detect帧:%d",
            KFs_.size(), vis_cnt, lidar_cnt, fusion_cnt, published_cnt,
            stat_pcd_frames_, stat_detect_frames_);
        stat_pcd_frames_ = 0;
        stat_detect_frames_ = 0;
    }

    // 将无人机 KF 追加到 Locations 消息
    void appendAirLocations(detect_result::msg::Locations& lm) {
        std::lock_guard<std::mutex> lock(kf_air_mutex_);
        for (auto& kf : KFs_air_) {
            if (kf.hits < 3) continue;
            float x = kf.predict_point.x;
            float y = kf.predict_point.y;
            // 过滤漂出场外的杂点 KF
            if (x < -2.0f || x > 30.0f || y < -2.0f || y > 17.0f) continue;
            detect_result::msg::Location loc;
            loc.x  = x;
            loc.y  = y;
            loc.z  = 2.0f;
            // per-KF 迟滞: x<13.5→Red, x>14.5→Blue, 中间保持
            char& c = air_color_cache_[&kf];
            if (x < 13.5f)       c = 'R';
            else if (x > 14.5f)  c = 'B';
            else if (!c)         c = (x < 14.0f) ? 'R' : 'B';  // 首次进入死区
            if (c == 'R') { loc.id = 6;   loc.label = "Red";  }
            else          { loc.id = 106; loc.label = "Blue"; }
            lm.locs.push_back(loc);
        }
    }

    // T-DT: callback — 点云主处理，严格对齐每一行
    void pcdCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        stat_pcd_frames_++;
        // T-DT: 点云转 PCL
        // 清空 detect 缓存（detectCb 已经做了 cameraMatch）
        {
            std::lock_guard<std::mutex> lock(detect_cache_mutex_);
            if (!detect_cache_.empty()) {
                // 保留最近一帧作为参考
                while (detect_cache_.size() > 1) detect_cache_.pop_front();
            }
        }
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        if (cloud.empty()) return;

        // DBSCAN 聚类（T-DT 的输入是聚类中心，这里需要对原始点云聚类）
        auto clus = cluster_->cluster(cloud);
        std::vector<pcl::PointXY> points;
        for (auto& pts : clus) {
            if (pts.empty()) continue;
            Eigen::Vector3f center(0,0,0);
            for (auto& p : pts) center += p;
            center /= pts.size();
            Eigen::Vector4d h(center(0), center(1), center(2), 1);
            auto w = r2f_ * h;
            pcl::PointXY pt;
            pt.x = w(0); pt.y = w(1);
            points.push_back(pt);
        }
        // ===== 单次加锁：predict → match(可选) → cleanup → publish =====
        // 即使无雷达聚类点，仍需 predict + cleanup + publish（纯视觉 KF 不应被雷达锁死）
        auto lm = std::make_unique<detect_result::msg::Locations>();
        {
            std::lock_guard<std::mutex> lock(kf_mutex_);

            // Step 1: 始终对所有 KF 做 predict
            for (auto& kf : KFs_) {
                kf.update_predict_point();
                kf.has_updated = false;
            }

            // Step 2: 仅当有雷达聚类点时做 match + update
            if (!points.empty()) {
                for (auto& pt : points) {
                    std::vector<int> match_indices;
                    for (int i = 0; i < (int)KFs_.size(); ++i) {
                        if (KFs_[i].match(pt))
                            match_indices.push_back(i);
                    }

                    if (match_indices.empty()) {
                        KalmanFilterPlus kf(pt, 0);
                        kf.delete_time   = kf_delete_time_;
                        kf.car_max_speed = kf_max_speed_;
                        kf.detect_r      = kf_detect_r_;
                        KFs_.push_back(kf);
                    } else if (match_indices.size() == 1) {
                        KFs_[match_indices[0]].update(pt);
                    } else {
                        float min_dist = 1e9f;
                        int min_index = match_indices[0];
                        for (int idx : match_indices) {
                            float d = std::sqrt(
                                std::pow(KFs_[idx].predict_point.x - pt.x, 2) +
                                std::pow(KFs_[idx].predict_point.y - pt.y, 2));
                            if (d < min_dist) { min_dist = d; min_index = idx; }
                        }
                        KFs_[min_index].update(pt);
                    }
                }
            }

            // Step 3: 始终清理超时 KF
            for (int i = (int)KFs_.size() - 1; i >= 0; --i) {
                if (KFs_[i].last_time > KFs_[i].delete_time) {
                    KFs_.erase(KFs_.begin() + i);
                }
            }

            // Step 4: 每个 KF 独立推入 Locations，不再按 number 槽位覆盖
            for (auto& kf : KFs_) {
                if (kf.detect_history.empty()) continue;
                int color  = kf.getColor();
                int number = kf.getNumber();
                if (number < 0 || number >= (int)labels_.size()) continue;
                std::string label;
                if (color == 0)      label = "B" + labels_[number];
                else if (color == 2) label = "R" + labels_[number];
                else continue;
                int cid = labelToCarId(label);
                if (cid == 0) continue;
                bool enemy = false, my7 = false;
                if (my_color_ == "Red") {
                    enemy = (cid >= 100);
                    my7   = (cid == 7);
                } else {
                    enemy = (cid < 100 && cid > 0);
                    my7   = (cid == 107);
                }
                if (debug_publish_all_ || enemy || my7) {
                    detect_result::msg::Location loc;
                    loc.x  = kf.predict_point.x;
                    loc.y  = kf.predict_point.y;
                    loc.z  = 0.0f;
                    loc.id = cid;
                    loc.label = (color == 2) ? "Red" : "Blue";
                    lm->locs.push_back(loc);
                }
            }
        }  // 单次锁结束
        appendAirLocations(*lm);
        if (!lm->locs.empty()) pub_loc_->publish(std::move(lm));
    }

    // ---- 无人机回调：聚类 → KF 跟踪 → 发布 ----
    void otherCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        if (cloud.empty()) return;

        // DBSCAN 聚类
        auto clus = cluster_air_->cluster(cloud);
        std::vector<pcl::PointXY> points;
        for (auto& pts : clus) {
            if (pts.empty()) continue;
            Eigen::Vector3f center(0, 0, 0);
            for (auto& p : pts) center += p;
            center /= pts.size();
            Eigen::Vector4d h(center(0), center(1), center(2), 1);
            auto w = r2f_ * h;
            pcl::PointXY pt;
            pt.x = w(0); pt.y = w(1);
            // 过滤场外杂点（margin ±2m）
            if (pt.x < -2.0f || pt.x > 30.0f || pt.y < -2.0f || pt.y > 17.0f) continue;
            points.push_back(pt);
        }
        if (points.empty()) return;

        // 更新无人机 KF
        {
            std::lock_guard<std::mutex> lock(kf_air_mutex_);

            for (auto& kf : KFs_air_) {
                kf.update_predict_point();
                kf.has_updated = false;
            }
            for (auto& pt : points) {
                std::vector<int> match_indices;
                for (int i = 0; i < (int)KFs_air_.size(); ++i) {
                    if (KFs_air_[i].match(pt)) match_indices.push_back(i);
                }
                if (match_indices.empty()) {
                    KalmanFilterPlus kf(pt, 0);
                    kf.delete_time   = kf_air_delete_time_;
                    kf.car_max_speed = kf_air_max_speed_;
                    kf.detect_r      = kf_air_detect_r_;
                    KFs_air_.push_back(kf);
                } else {
                    int best_i = match_indices[0];
                    float best_d = std::hypot(KFs_air_[best_i].predict_point.x - pt.x,
                                              KFs_air_[best_i].predict_point.y - pt.y);
                    for (size_t j = 1; j < match_indices.size(); ++j) {
                        float d = std::hypot(KFs_air_[match_indices[j]].predict_point.x - pt.x,
                                             KFs_air_[match_indices[j]].predict_point.y - pt.y);
                        if (d < best_d) { best_d = d; best_i = match_indices[j]; }
                    }
                    KFs_air_[best_i].update(pt);
                }
            }
            for (int i = (int)KFs_air_.size() - 1; i >= 0; --i) {
                if (KFs_air_[i].last_time > KFs_air_[i].delete_time) {
                    air_color_cache_.erase(&KFs_air_[i]);
                    KFs_air_.erase(KFs_air_.begin() + i);
                }
            }
        }

        // 构建合并消息（地面 + 无人机）
        auto lm = std::make_unique<detect_result::msg::Locations>();
        {
            std::lock_guard<std::mutex> lock(kf_mutex_);
            for (auto& kf : KFs_) {
                if (kf.detect_history.empty()) continue;
                int color  = kf.getColor();
                int number = kf.getNumber();
                if (number < 0 || number >= (int)labels_.size()) continue;
                std::string label;
                if (color == 0)      label = "B" + labels_[number];
                else if (color == 2) label = "R" + labels_[number];
                else continue;
                int cid = labelToCarId(label);
                if (cid == 0) continue;
                bool enemy = false, my7 = false;
                if (my_color_ == "Red") {
                    enemy = (cid >= 100); my7 = (cid == 7);
                } else {
                    enemy = (cid < 100 && cid > 0); my7 = (cid == 107);
                }
                if (debug_publish_all_ || enemy || my7) {
                    detect_result::msg::Location loc;
                    loc.x  = kf.predict_point.x;
                    loc.y  = kf.predict_point.y;
                    loc.z  = 0.0f;
                    loc.id = cid;
                    loc.label = (color == 2) ? "Red" : "Blue";
                    lm->locs.push_back(loc);
                }
            }
        }
        appendAirLocations(*lm);
        if (!lm->locs.empty()) pub_loc_->publish(std::move(lm));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarNode>());
    rclcpp::shutdown();
    return 0;
}
