#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <detect_result/msg/robots.hpp>
#include <detect_result/msg/detect_result.hpp>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

#include <thread>
#include <mutex>
#include <deque>
#include <unordered_map>
#include <map>
#include <condition_variable>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <cstring>
#include <atomic>

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include "hnurm_radar/infer.hpp"
#include "hnurm_radar/HomographyTransformer.hpp"
#include "hnurm_radar/tracker.hpp"

using namespace std::chrono_literals;

class DetectorNode : public rclcpp::Node {
    struct DisplayTask {
        cv::Mat frame;
        std::vector<InferArmor> armors;
        std::vector<std::pair<float, float>> coords;
    };

public:
    DetectorNode() : Node("detector") {
        RCLCPP_INFO(get_logger(), "CWD: %s", std::filesystem::current_path().string().c_str());

        YAML::Node det_cfg = YAML::LoadFile("configs/detector_config.yaml");
        YAML::Node main_cfg = YAML::LoadFile("configs/main_config.yaml");

        my_color_ = main_cfg["global"]["my_color"].as<std::string>("Red");
        camera_mode_ = main_cfg["camera"]["mode"].as<std::string>("hik");

        std::string stage1_path = det_cfg["path"]["stage_one_path"].as<std::string>();
        std::string stage2_path = det_cfg["path"]["stage_two_path"].as<std::string>();
        std::string stage3_path = det_cfg["path"]["stage_three_path"].as<std::string>();

        conf_car_   = det_cfg["params"]["stage_one_conf"].as<float>();
        conf_armor_ = det_cfg["params"]["stage_two_conf"].as<float>();
        labels_ = det_cfg["params"]["labels"].as<std::vector<std::string>>();

        // 初始化推理引擎
        infer_engine_ = std::make_unique<InferEngine>(stage1_path, stage2_path, stage3_path,
                                                      conf_car_, conf_armor_, labels_);

        // 透视变换
        std::string calib_json = "configs/perspective_calib.json";
        std::string mask_img_path = "data/maps/competition_2026/pfa_map_mask_2025.jpg";
        homography_ = std::make_shared<HomographyTransformer>(calib_json, mask_img_path);
    obj_tracker_ = std::make_unique<ObjectTracker>(0.3f, 30, 3);

        // 小地图
        std::string scene = main_cfg["global"]["scene"].as<std::string>("competition");
        std::string minimap_rel = main_cfg["scenes"][scene]["pfa_map"].as<std::string>();
        if (!minimap_rel.empty()) {
            minimap_img_ = cv::imread(minimap_rel);
            if (minimap_img_.empty()) {
                RCLCPP_WARN(get_logger(), "小地图加载失败: %s", minimap_rel.c_str());
            } else {
                cv::namedWindow("MiniMap", cv::WINDOW_NORMAL);
                cv::resizeWindow("MiniMap", 800, 471);
            }
        }

        cv::namedWindow("Detector", cv::WINDOW_NORMAL);
        cv::resizeWindow("Detector", 1280, 720);

        rclcpp::QoS qos(rclcpp::KeepLast(3), rmw_qos_profile_sensor_data);

        if (camera_mode_ == "hik") {
            init_hik_shared_memory();
            hik_thread_ = std::thread(&DetectorNode::hik_worker, this);
        } else {
            if (camera_mode_ == "rosbag") {
                std::string topic = main_cfg["camera"]["compressed_image_topic"].as<std::string>("/compressed_image");
                sub_compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                    topic, qos, std::bind(&DetectorNode::compressed_callback, this, std::placeholders::_1));
            } else {
                sub_image_ = create_subscription<sensor_msgs::msg::Image>(
                    "image", qos, std::bind(&DetectorNode::image_callback, this, std::placeholders::_1));
            }
            for (int i = 0; i < 1; ++i) {
                decode_threads_.emplace_back(&DetectorNode::decode_worker, this);
            }
        }

        pub_result_ = create_publisher<detect_result::msg::Robots>("detect_result", qos);
        pub_view_   = create_publisher<sensor_msgs::msg::Image>("detect_view", qos);

        timer_ = create_wall_timer(5s, std::bind(&DetectorNode::print_stats, this));

        process_thread_ = std::thread(&DetectorNode::process_loop, this);
        display_thread_ = std::thread(&DetectorNode::display_loop, this);
    }

    ~DetectorNode() {
        stop_hik_ = true;
        if (hik_thread_.joinable()) hik_thread_.join();
        release_hik_shared_memory();

        stop_decode_ = true;
        decode_cv_.notify_all();
        for (auto& t : decode_threads_) if (t.joinable()) t.join();

        stop_process_ = true;
        process_cv_.notify_all();
        if (process_thread_.joinable()) process_thread_.join();

        stop_display_ = true;
        display_cv_.notify_all();
        if (display_thread_.joinable()) display_thread_.join();
    }

private:
    // ---------- Hik 共享内存 ----------
    void init_hik_shared_memory() {
        const char* shm_name = "/hik_camera";
        int fd = shm_open(shm_name, O_RDONLY, 0666);
        if (fd < 0) {
            RCLCPP_ERROR(get_logger(), "shm_open failed: %s", strerror(errno));
            return;
        }
        size_t header_size = sizeof(int32_t)*2 + sizeof(uint32_t) + sizeof(int64_t);
        void* p = mmap(nullptr, header_size, PROT_READ, MAP_SHARED, fd, 0);
        close(fd);
        if (p == MAP_FAILED) {
            RCLCPP_ERROR(get_logger(), "mmap header failed");
            return;
        }
        int w = *reinterpret_cast<int32_t*>((uint8_t*)p);
        int h = *reinterpret_cast<int32_t*>((uint8_t*)p + 4);
        munmap(p, header_size);

        if (w <= 0 || h <= 0) {
            RCLCPP_ERROR(get_logger(), "Invalid shared memory size: %dx%d", w, h);
            return;
        }

        uint32_t encoding_len = 4;  // "bgr8"
        shm_size_ = header_size + sizeof(uint32_t) + encoding_len + w * h * 3;

        fd = shm_open(shm_name, O_RDONLY, 0666);
        shm_ptr_ = mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED, fd, 0);
        close(fd);
        if (shm_ptr_ == MAP_FAILED) {
            RCLCPP_ERROR(get_logger(), "mmap full failed");
            shm_ptr_ = nullptr;
            return;
        }

        hik_img_width_ = w;
        hik_img_height_ = h;
        last_frame_index_ = 0xFFFFFFFF;
        RCLCPP_INFO(get_logger(), "Hik shared memory ready: %dx%d", w, h);
    }

    void release_hik_shared_memory() {
        if (shm_ptr_ && shm_ptr_ != MAP_FAILED) {
            munmap(shm_ptr_, shm_size_);
            shm_ptr_ = nullptr;
        }
    }

    void hik_worker() {
        while (rclcpp::ok() && !stop_hik_) {
            if (!shm_ptr_) {
                std::this_thread::sleep_for(10ms);
                continue;
            }
            auto* header = static_cast<uint8_t*>(shm_ptr_);
            uint32_t frame_idx = *reinterpret_cast<uint32_t*>(header + 8);
            if (frame_idx != last_frame_index_) {
                last_frame_index_ = frame_idx;
                auto t_read_start = std::chrono::steady_clock::now();

                size_t header_sz = sizeof(int32_t)*2 + sizeof(uint32_t) + sizeof(int64_t);
                size_t enc_sz = sizeof(uint32_t) + 4;
                uint8_t* img_data = header + header_sz + enc_sz;
                cv::Mat img(hik_img_height_, hik_img_width_, CV_8UC3, img_data);
                if (!img.empty()) {
                    cv::Mat cloned = img.clone();
                    auto t_read_end = std::chrono::steady_clock::now();
                    double read_ms = std::chrono::duration<double, std::milli>(t_read_end - t_read_start).count();

                    {
                        std::lock_guard<std::mutex> lock(process_mutex_);
                        process_queue_.push_back(std::move(cloned));
                    }
                    process_cv_.notify_one();

                    hik_read_time_ += read_ms;
                    hik_read_count_++;
                }
            }
            std::this_thread::sleep_for(1ms);
        }
    }

    // ---------- 回调函数（非 hik 模式）----------
    void compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(decode_mutex_);
            decode_queue_.push_back(msg);
        }
        decode_cv_.notify_one();
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if (!frame.empty()) {
            std::lock_guard<std::mutex> lock(process_mutex_);
            process_queue_.push_back(std::move(frame));
            process_cv_.notify_one();
        }
    }

    // ---------- 解码工作线程 ----------
    void decode_worker() {
        while (rclcpp::ok() && !stop_decode_) {
            std::unique_lock<std::mutex> lock(decode_mutex_);
            decode_cv_.wait(lock, [this] { return !decode_queue_.empty() || stop_decode_; });
            if (stop_decode_) break;
            auto msg = std::move(decode_queue_.front());
            decode_queue_.pop_front();
            lock.unlock();

            cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (!frame.empty()) {
                std::lock_guard<std::mutex> plock(process_mutex_);
                process_queue_.push_back(std::move(frame));
                process_cv_.notify_one();
            }
        }
    }

    // ---------- 主处理线程 ----------
    void process_loop() {
        while (rclcpp::ok() && !stop_process_) {
            cv::Mat frame;
            {
                std::unique_lock<std::mutex> lock(process_mutex_);
                // 先检查队列是否有帧
                if (process_queue_.empty()) {
                    // 没有帧就释放锁等 notify，但设 2ms 超时防止死等
                    process_cv_.wait_for(lock, 2ms);
                }
                if (process_queue_.empty()) {
                    continue;
                }
                // 取最新帧，丢弃积压的旧帧
                frame = std::move(process_queue_.back());
                process_queue_.clear();
            }
            process_frame(frame);
        }
    }

    // ---------- 核心处理 ----------
    void process_frame(cv::Mat frame) {
        auto t_total = std::chrono::steady_clock::now();

        // 调用推理引擎
        auto armors = infer_engine_->infer(frame);

        // 获取车辆框（1280坐标）用于绘制
        const auto& car_boxes = infer_engine_->getLastCarBoxes();

        // 准备1280尺寸图像用于绘制
        cv::Mat frame_resized;
        cv::resize(frame, frame_resized, cv::Size(1280, 1280));
        float scale_x = static_cast<float>(frame.cols) / 1280.0f;
        float scale_y = static_cast<float>(frame.rows) / 1280.0f;

        // 绘制车辆框
        for (const auto& box : car_boxes) {
            cv::rectangle(frame_resized, box, cv::Scalar(255, 0, 0), 2);
        }

        // 计算场地坐标并绘制装甲板框
        std::vector<std::pair<float, float>> coords;
        for (auto& ar : armors) {
            // 场地坐标
            auto field_xy = homography_->pixelToField(ar.car_bottom_x, ar.car_bottom_y, my_color_);
            if (field_xy.first >= 0) {
                ar.field_x = field_xy.first;
                ar.field_y = field_xy.second;
                ar.valid_field = true;
                coords.emplace_back(field_xy.first, field_xy.second);
            }

            // 在1280图像上绘制装甲板框（反向缩放）
            int draw_x1 = static_cast<int>(ar.abs_rect.x / scale_x);
            int draw_y1 = static_cast<int>(ar.abs_rect.y / scale_y);
            int draw_x2 = static_cast<int>((ar.abs_rect.x + ar.abs_rect.width) / scale_x);
            int draw_y2 = static_cast<int>((ar.abs_rect.y + ar.abs_rect.height) / scale_y);
            cv::rectangle(frame_resized, cv::Point(draw_x1, draw_y1), cv::Point(draw_x2, draw_y2),
                          cv::Scalar(0, 255, 0), 2);
            cv::putText(frame_resized, ar.label, cv::Point(draw_x1, draw_y1 - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        // ===== 帧间追踪器（对车辆框做追踪，每个 track 代表一辆车） =====
        // 获取车辆底部中心点（1280x1280坐标系）
        const auto& car_bottoms = infer_engine_->getLastCarBottomPts();

        // 用车辆框做追踪
        std::vector<cv::Rect> car_track_rects;
        std::vector<std::string> car_track_labels;
        std::vector<cv::Point2f> car_track_bottoms;
        for (size_t i = 0; i < car_boxes.size(); ++i) {
            car_track_rects.push_back(car_boxes[i]);
            car_track_labels.push_back("car");
            if (i < car_bottoms.size()) {
                car_track_bottoms.emplace_back(car_bottoms[i].first, car_bottoms[i].second);
            } else {
                car_track_bottoms.emplace_back(
                    (car_boxes[i].x + car_boxes[i].width / 2.0f),
                    (car_boxes[i].y + car_boxes[i].height)
                );
            }
        }

        // 追踪器更新，得到带 track_id 的车辆轨迹
        auto car_tracked = obj_tracker_->update(car_track_rects, car_track_labels, car_track_bottoms);

        // 将车辆 track_id 回写到每个 armor
        for (auto& ar : armors) {
            int best_match = -1;
            float best_iou = 0.0f;
            float ar_cx = (ar.abs_rect.x + ar.abs_rect.width / 2.0f) / scale_x;
            float ar_cy = (ar.abs_rect.y + ar.abs_rect.height / 2.0f) / scale_y;
            cv::Point2f ar_center(ar_cx, ar_cy);

            for (const auto& tr : car_tracked) {
                if (tr.last_rect.contains(ar_center)) {
                    cv::Rect armor_in_1280(
                        ar.abs_rect.x / scale_x,
                        ar.abs_rect.y / scale_y,
                        ar.abs_rect.width / scale_x,
                        ar.abs_rect.height / scale_y
                    );
                    cv::Rect inter = tr.last_rect & armor_in_1280;
                    float iou_val = (float)inter.area() / (float)(tr.last_rect.area() + armor_in_1280.area() - inter.area());
                    if (iou_val > best_iou) {
                        best_iou = iou_val;
                        best_match = tr.track_id;
                    }
                }
            }
            ar.track_id = best_match;
        }

        // T-DT方式：按 track_id 分组，每辆车只选置信度最高且数字非0非5的 armor
        std::map<int, const InferArmor*> best_armor_per_car;
        for (const auto& ar : armors) {
            if (ar.label.size() < 2) continue;
            std::string num_part = ar.label.substr(1);
            int digit_idx = -1;
            for (int i = 0; i < (int)labels_.size(); ++i) {
                if (labels_[i] == num_part) { digit_idx = i; break; }
            }
            if (digit_idx < 0 || ar.track_id < 0) continue;
            auto it = best_armor_per_car.find(ar.track_id);
            if (it == best_armor_per_car.end() || ar.conf > it->second->conf) {
                best_armor_per_car[ar.track_id] = &ar;
            }
        }

        // 颜色+数字判定：逐帧观察 → 时序多数投票，消除突变
        detect_result::msg::Robots robots_msg;
        // 清理消失的 track 历史：连续 miss >= 5 帧才删除
        for (auto it = track_histories_.begin(); it != track_histories_.end(); ) {
            if (best_armor_per_car.find(it->first) == best_armor_per_car.end()) {
                it->second.markMissed();
                if (it->second.missed >= 5)
                    it = track_histories_.erase(it);
                else ++it;
            } else ++it;
        }
        for (auto& [tid, ar] : best_armor_per_car) {
            cv::Rect safe_rect = ar->abs_rect & cv::Rect(0, 0, frame.cols, frame.rows);
            if (safe_rect.width <= 0 || safe_rect.height <= 0) continue;
            cv::Mat armor_roi = frame(safe_rect);
            std::vector<cv::Mat> channels;
            cv::split(armor_roi, channels);
            cv::Mat blueMinusRed = channels[0] - channels[2];
            cv::Mat redMinusBlue = channels[2] - channels[0];
            double avgBMR = cv::mean(blueMinusRed)[0];
            double avgRMB = cv::mean(redMinusBlue)[0];
            char per_frame_color = (avgBMR > avgRMB) ? 'B' : 'R';
            std::string per_frame_digit = (ar->label.size() >= 2) ? ar->label.substr(1) : "";

            // 写入时序历史
            auto& hist = track_histories_[tid];
            hist.add(per_frame_color, per_frame_digit);

            // 用多数投票取稳定值
            char stable_color = hist.stableColor();
            std::string stable_digit = hist.stableDigit();
            std::string final_label = stable_color + stable_digit;

            detect_result::msg::DetectResult dr;
            dr.xyxy_box = {ar->abs_rect.x, ar->abs_rect.y,
                           ar->abs_rect.x + ar->abs_rect.width,
                           ar->abs_rect.y + ar->abs_rect.height};
            dr.label = final_label;
            dr.field_x = ar->field_x;
            dr.field_y = ar->field_y;
            robots_msg.detect_results.push_back(dr);
        }
        robots_msg_count_ += robots_msg.detect_results.size();
        pub_result_->publish(robots_msg);

        // 显示任务
        DisplayTask task;
        task.frame = frame_resized.clone();
        task.armors = armors;
        task.coords = coords;
        {
            std::lock_guard<std::mutex> lock(display_mutex_);
            if (display_queue_.size() < 3) {
                display_queue_.push_back(std::move(task));
            }
        }
        display_cv_.notify_one();

        // 细粒度统计累加
        auto t_processed = std::chrono::steady_clock::now();
        double total_ms = std::chrono::duration<double, std::milli>(t_processed - t_total).count();

        car_pre_time_  += infer_engine_->getLastCarPreprocessTime();
        car_inf_time_  += infer_engine_->getLastCarInferTime();
        car_post_time_ += infer_engine_->getLastCarPostprocessTime();
        armor_pre_time_  += infer_engine_->getLastArmorPreprocessTime();
        armor_post_time_ += infer_engine_->getLastArmorPostprocessTime();
        digit_time_ += infer_engine_->getLastDigitTotalTime();
        frame_time_ += total_ms;
        frame_count_++;
    }

    // ---------- 显示线程 ----------
    void display_loop() {
        while (rclcpp::ok() && !stop_display_) {
            std::unique_lock<std::mutex> lock(display_mutex_);
            display_cv_.wait(lock, [this] { return !display_queue_.empty() || stop_display_; });
            if (stop_display_) break;
            DisplayTask task = std::move(display_queue_.front());
            display_queue_.pop_front();
            lock.unlock();

            cv::Mat disp;
            double scale = std::min(1280.0 / task.frame.cols, 720.0 / task.frame.rows);
            cv::resize(task.frame, disp, cv::Size(task.frame.cols * scale, task.frame.rows * scale));

            static auto last_show = std::chrono::steady_clock::now();
            auto now_show = std::chrono::steady_clock::now();
            double fps = 1.0e3 / std::chrono::duration<double, std::milli>(now_show - last_show).count();
            last_show = now_show;
            cv::putText(disp, cv::format("FPS: %.1f", fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

            pub_view_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", task.frame).toImageMsg());
            cv::imshow("Detector", disp);
            cv::waitKey(1);

            if (!minimap_img_.empty()) {
                cv::Mat minimap = minimap_img_.clone();
                for (size_t i = 0; i < task.coords.size(); ++i) {
                    float fx = task.coords[i].first, fy = task.coords[i].second;
                    int map_xx = static_cast<int>(std::round(fx * 100));
                    int map_yy = static_cast<int>(std::round(1500 - fy * 100));
                    map_xx = std::clamp(map_xx, 0, 2800);
                    map_yy = std::clamp(map_yy, 0, 1500);
                    cv::Scalar color = (task.armors[i].label[0] == 'R') ? cv::Scalar(0, 0, 255) : cv::Scalar(250, 100, 0);
                    cv::circle(minimap, cv::Point(map_xx, map_yy), 40, color, 3);
                    cv::putText(minimap, task.armors[i].label, cv::Point(map_xx - 15, map_yy - 20),
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
                }
                cv::imshow("MiniMap", minimap);
                cv::waitKey(1);
            }
        }
    }

    void print_stats() {
        if (frame_count_ == 0) return;
        RCLCPP_INFO(get_logger(),
            "[Detect统计] 过去5秒处理帧数:%d  平均FPS:%.1f  detect_result发布数:%d",
            frame_count_, frame_count_ / 5.0, (int)robots_msg_count_);
        double avg_car_pre  = car_pre_time_  / frame_count_;
        double avg_car_inf  = car_inf_time_  / frame_count_;
        double avg_car_post = car_post_time_ / frame_count_;
        double avg_armor_pre  = armor_pre_time_  / frame_count_;
        double avg_armor_post = armor_post_time_ / frame_count_;
        double avg_digit = digit_time_ / frame_count_;
        double avg_frame = frame_time_ / frame_count_;

        RCLCPP_INFO(get_logger(),
            "===== 过去5秒性能统计 (帧数: %d, FPS: %.1f) =====", frame_count_, frame_count_/5.0);
        RCLCPP_INFO(get_logger(),
            "  [Stage1 整车]  预处理:%5.1fms | 推理:%5.1fms | 后处理:%5.1fms | 合计:%5.1fms",
            avg_car_pre, avg_car_inf, avg_car_post, avg_car_pre+avg_car_inf+avg_car_post);
        RCLCPP_INFO(get_logger(),
            "  [Stage2 装甲]  预处理+推理:%5.1fms | 后处理(坐标):%5.1fms",
            avg_armor_pre, avg_armor_post);
        RCLCPP_INFO(get_logger(),
            "  [Stage3 数字]  总耗时:%5.1fms", avg_digit);
        RCLCPP_INFO(get_logger(),
            "  [总循环]       %.1fms", avg_frame);

        if (camera_mode_ == "hik" && hik_read_count_ > 0) {
            RCLCPP_INFO(get_logger(),
                "  [Hik共享内存] 读取平均: %.2fms (帧数 %d)",
                hik_read_time_ / hik_read_count_, hik_read_count_);
        }

        // 重置计数器
        frame_count_ = 0;
        robots_msg_count_ = 0;
        car_pre_time_ = car_inf_time_ = car_post_time_ = 0.0;
        armor_pre_time_ = armor_post_time_ = digit_time_ = 0.0;
        frame_time_ = 0.0;
        hik_read_time_ = 0.0;
        hik_read_count_ = 0;
    }

    // 成员变量
    std::string my_color_, camera_mode_;
    float conf_car_, conf_armor_;
    std::vector<std::string> labels_;

    std::unique_ptr<InferEngine> infer_engine_;
    std::unique_ptr<ObjectTracker> obj_tracker_;
    std::shared_ptr<HomographyTransformer> homography_;
    cv::Mat minimap_img_;

    // 时序颜色/数字过滤：消除逐帧突变
    struct TrackHistory {
        std::deque<char> colors;
        std::deque<std::string> digits;
        int missed = 0;  // 连续未出现帧数，>=5 才清理
        static constexpr size_t MAX_HISTORY = 7;
        void add(char c, const std::string& d) {
            colors.push_back(c);
            digits.push_back(d);
            if (colors.size() > MAX_HISTORY) { colors.pop_front(); digits.pop_front(); }
            missed = 0;
        }
        void markMissed() { missed++; }
        char stableColor() const {
            int r = 0, b = 0;
            for (char c : colors) { if (c == 'R') r++; else if (c == 'B') b++; }
            return (r > b) ? 'R' : 'B';
        }
        std::string stableDigit() const {
            std::map<std::string, int> cnt;
            for (auto& d : digits) cnt[d]++;
            std::string best; int bc = 0;
            for (auto& [d, c] : cnt) if (c > bc) { bc = c; best = d; }
            return best;
        }
    };
    std::unordered_map<int, TrackHistory> track_histories_;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<detect_result::msg::Robots>::SharedPtr pub_result_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_view_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 解码队列
    std::deque<sensor_msgs::msg::CompressedImage::SharedPtr> decode_queue_;
    std::mutex decode_mutex_;
    std::condition_variable decode_cv_;
    std::vector<std::thread> decode_threads_;
    bool stop_decode_ = false;

    // 处理队列
    std::deque<cv::Mat> process_queue_;
    std::mutex process_mutex_;
    std::condition_variable process_cv_;
    std::thread process_thread_;
    bool stop_process_ = false;

    // 显示队列
    std::deque<DisplayTask> display_queue_;
    std::mutex display_mutex_;
    std::condition_variable display_cv_;
    std::thread display_thread_;
    bool stop_display_ = false;

    // Hik 共享内存
    void* shm_ptr_ = nullptr;
    size_t shm_size_ = 0;
    int hik_img_width_ = 0, hik_img_height_ = 0;
    uint32_t last_frame_index_ = 0;
    std::thread hik_thread_;
    bool stop_hik_ = false;

    // 细粒度统计累加器
    int frame_count_ = 0;
    int robots_msg_count_ = 0;
    double car_pre_time_ = 0.0, car_inf_time_ = 0.0, car_post_time_ = 0.0;
    double armor_pre_time_ = 0.0, armor_post_time_ = 0.0, digit_time_ = 0.0;
    double frame_time_ = 0.0;
    double hik_read_time_ = 0.0;
    int hik_read_count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectorNode>());
    rclcpp::shutdown();
    return 0;
}
