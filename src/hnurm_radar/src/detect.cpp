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
            for (int i = 0; i < 4; ++i) {
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
            std::unique_lock<std::mutex> lock(process_mutex_);
            process_cv_.wait(lock, [this] { return !process_queue_.empty() || stop_process_; });
            if (stop_process_) break;
            cv::Mat frame = std::move(process_queue_.front());
            process_queue_.pop_front();
            lock.unlock();

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
        // 获取内部耗时用于统计
        float car_ms = infer_engine_->getLastCarTime();
        float armor_ms = infer_engine_->getLastArmorTime();
        float total_infer_ms = infer_engine_->getLastTotalTime();

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

        // 发布检测结果
        detect_result::msg::Robots robots_msg;
        for (const auto& ar : armors) {
            detect_result::msg::DetectResult dr;
            dr.xyxy_box = {ar.abs_rect.x, ar.abs_rect.y,
                           ar.abs_rect.x + ar.abs_rect.width,
                           ar.abs_rect.y + ar.abs_rect.height};
            dr.label = ar.label;
            dr.field_x = ar.field_x;
            dr.field_y = ar.field_y;
            robots_msg.detect_results.push_back(dr);
        }
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

        // 统计
        auto t_processed = std::chrono::steady_clock::now();
        double total_ms = std::chrono::duration<double, std::milli>(t_processed - t_total).count();
        car_time_ += car_ms;
        armor_time_ += armor_ms;
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
        double avg_car = car_time_ / frame_count_;
        double avg_armor = armor_time_ / frame_count_;
        double avg_frame = frame_time_ / frame_count_;
        RCLCPP_INFO(get_logger(),
            "过去5秒: 帧数=%d | 整车+预处理 %.1fms | 装甲板推理 %.1fms | 总循环耗时 %.1fms | FPS %.1f",
            frame_count_, avg_car, avg_armor, avg_frame, frame_count_ / 5.0);
        if (camera_mode_ == "hik" && hik_read_count_ > 0) {
            RCLCPP_INFO(get_logger(),
                "Hik 共享内存读取平均耗时: %.2fms (帧数 %d)",
                hik_read_time_ / hik_read_count_, hik_read_count_);
        }
        frame_count_ = 0;
        car_time_ = armor_time_ = frame_time_ = 0.0;
        hik_read_time_ = 0.0;
        hik_read_count_ = 0;
    }

    // 成员变量
    std::string my_color_, camera_mode_;
    float conf_car_, conf_armor_;
    std::vector<std::string> labels_;

    std::unique_ptr<InferEngine> infer_engine_;
    std::shared_ptr<HomographyTransformer> homography_;
    cv::Mat minimap_img_;

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

    int frame_count_ = 0;
    double car_time_ = 0.0, armor_time_ = 0.0, frame_time_ = 0.0;
    double hik_read_time_ = 0.0;
    int hik_read_count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectorNode>());
    rclcpp::shutdown();
    return 0;
}