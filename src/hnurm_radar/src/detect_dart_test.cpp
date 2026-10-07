#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;

class DetectDart : public rclcpp::Node {
public:
    DetectDart() : Node("detect_dart") {
        RCLCPP_INFO(get_logger(), "CWD: %s", std::filesystem::current_path().string().c_str());

        std::string topic = "/compressed_image";
        try {
            YAML::Node main_cfg = YAML::LoadFile("configs/main_config.yaml");
            topic = main_cfg["camera"]["compressed_image_topic"].as<std::string>(topic);
        } catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(), "Failed to load configs/main_config.yaml, fallback topic: %s, reason: %s",
                        topic.c_str(), e.what());
        }

        rclcpp::QoS qos(rclcpp::KeepLast(3), rmw_qos_profile_sensor_data);
        sub_compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            topic,
            qos,
            std::bind(&DetectDart::compressed_callback, this, std::placeholders::_1));

        cv::namedWindow("DetectDartRaw", cv::WINDOW_NORMAL);
        cv::resizeWindow("DetectDartRaw", 1280, 720);
        cv::namedWindow("DetectDartExtract", cv::WINDOW_NORMAL);
        cv::resizeWindow("DetectDartExtract", 1280, 720);
        cv::namedWindow("DetectDartCtrl", cv::WINDOW_NORMAL);
        cv::resizeWindow("DetectDartCtrl", 640, 420);

        cv::createTrackbar("H1 Low", "DetectDartCtrl", nullptr, 179);
        cv::createTrackbar("H1 High", "DetectDartCtrl", nullptr, 179);
        cv::createTrackbar("H2 Low", "DetectDartCtrl", nullptr, 179);
        cv::createTrackbar("H2 High", "DetectDartCtrl", nullptr, 179);
        cv::createTrackbar("S Low", "DetectDartCtrl", nullptr, 255);
        cv::createTrackbar("V Low", "DetectDartCtrl", nullptr, 255);
        cv::createTrackbar("V Glow", "DetectDartCtrl", nullptr, 255);
        cv::createTrackbar("Red Bias", "DetectDartCtrl", nullptr, 100);
        cv::createTrackbar("Area Min", "DetectDartCtrl", nullptr, 4000);
        cv::createTrackbar("Morph", "DetectDartCtrl", nullptr, 21);
        cv::createTrackbar("Scale%", "DetectDartCtrl", nullptr, 100);
        cv::createTrackbar("Sq Area Min", "DetectDartCtrl", nullptr, 2000);
        cv::createTrackbar("Sq Area Max", "DetectDartCtrl", nullptr, 20000);
        cv::createTrackbar("Sq AR Tol%", "DetectDartCtrl", nullptr, 100);

        cv::setTrackbarPos("H1 Low", "DetectDartCtrl", h1_low_);
        cv::setTrackbarPos("H1 High", "DetectDartCtrl", h1_high_);
        cv::setTrackbarPos("H2 Low", "DetectDartCtrl", h2_low_);
        cv::setTrackbarPos("H2 High", "DetectDartCtrl", h2_high_);
        cv::setTrackbarPos("S Low", "DetectDartCtrl", s_low_);
        cv::setTrackbarPos("V Low", "DetectDartCtrl", v_low_);
        cv::setTrackbarPos("V Glow", "DetectDartCtrl", v_glow_low_);
        cv::setTrackbarPos("Red Bias", "DetectDartCtrl", red_bias_);
        cv::setTrackbarPos("Area Min", "DetectDartCtrl", area_min_);
        cv::setTrackbarPos("Morph", "DetectDartCtrl", morph_size_);
        cv::setTrackbarPos("Scale%", "DetectDartCtrl", proc_scale_);
        cv::setTrackbarPos("Sq Area Min", "DetectDartCtrl", square_area_min_);
        cv::setTrackbarPos("Sq Area Max", "DetectDartCtrl", square_area_max_);
        cv::setTrackbarPos("Sq AR Tol%", "DetectDartCtrl", square_ar_tol_percent_);

        stat_timer_ = create_wall_timer(5s, std::bind(&DetectDart::print_stats, this));
        RCLCPP_INFO(get_logger(), "DetectDart started, subscribe rosbag compressed image topic: %s", topic.c_str());
    }

    ~DetectDart() override {
        cv::destroyWindow("DetectDartRaw");
        cv::destroyWindow("DetectDartExtract");
        cv::destroyWindow("DetectDartCtrl");
    }

private:
    void compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) {
            return;
        }

        int h1_low = std::clamp(cv::getTrackbarPos("H1 Low", "DetectDartCtrl"), 0, 179);
        int h1_high = std::clamp(cv::getTrackbarPos("H1 High", "DetectDartCtrl"), 0, 179);
        int h2_low = std::clamp(cv::getTrackbarPos("H2 Low", "DetectDartCtrl"), 0, 179);
        int h2_high = std::clamp(cv::getTrackbarPos("H2 High", "DetectDartCtrl"), 0, 179);
        int s_low = std::clamp(cv::getTrackbarPos("S Low", "DetectDartCtrl"), 0, 255);
        int v_low = std::clamp(cv::getTrackbarPos("V Low", "DetectDartCtrl"), 0, 255);
        int v_glow_low = std::clamp(cv::getTrackbarPos("V Glow", "DetectDartCtrl"), 0, 255);
        int red_bias = std::clamp(cv::getTrackbarPos("Red Bias", "DetectDartCtrl"), 0, 100);
        int area_min = std::max(cv::getTrackbarPos("Area Min", "DetectDartCtrl"), 0);
        int proc_scale = std::clamp(cv::getTrackbarPos("Scale%", "DetectDartCtrl"), 20, 100);
        int sq_area_min = std::max(cv::getTrackbarPos("Sq Area Min", "DetectDartCtrl"), 1);
        int sq_area_max = std::max(cv::getTrackbarPos("Sq Area Max", "DetectDartCtrl"), sq_area_min + 1);
        int sq_ar_tol_percent = std::clamp(cv::getTrackbarPos("Sq AR Tol%", "DetectDartCtrl"), 1, 100);

        if (h1_low > h1_high) std::swap(h1_low, h1_high);
        if (h2_low > h2_high) std::swap(h2_low, h2_high);

        cv::Mat proc_frame;
        if (proc_scale < 100) {
            double s = static_cast<double>(proc_scale) / 100.0;
            cv::resize(frame, proc_frame, cv::Size(), s, s, cv::INTER_AREA);
        } else {
            proc_frame = frame;
        }

        cv::Mat hsv;
        cv::cvtColor(proc_frame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask_red_1, mask_red_2, mask_hsv;

        cv::inRange(hsv, cv::Scalar(h1_low, s_low, v_low), cv::Scalar(h1_high, 255, 255), mask_red_1);
        cv::inRange(hsv, cv::Scalar(h2_low, s_low, v_low), cv::Scalar(h2_high, 255, 255), mask_red_2);
        cv::bitwise_or(mask_red_1, mask_red_2, mask_hsv);

        std::vector<cv::Mat> bgr_channels;
        cv::split(proc_frame, bgr_channels);
        cv::Mat b = bgr_channels[0], g = bgr_channels[1], r = bgr_channels[2];

        cv::Mat max_bg, bg_plus, v_channel;
        cv::max(b, g, max_bg);
        cv::add(max_bg, cv::Scalar(red_bias), bg_plus);
        cv::extractChannel(hsv, v_channel, 2);

        cv::Mat mask_r_dom, mask_v;
        cv::compare(r, bg_plus, mask_r_dom, cv::CMP_GT);
        cv::inRange(v_channel, cv::Scalar(v_glow_low), cv::Scalar(255), mask_v);

        cv::Mat mask_red_glow;
        cv::bitwise_and(mask_hsv, mask_r_dom, mask_red_glow);
        cv::bitwise_and(mask_red_glow, mask_v, mask_red_glow);

        int kernel_size = std::max(1, cv::getTrackbarPos("Morph", "DetectDartCtrl"));
        if (kernel_size % 2 == 0) kernel_size += 1;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
        cv::morphologyEx(mask_red_glow, mask_red_glow, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask_red_glow, mask_red_glow, cv::MORPH_OPEN, kernel);

        // 连通域面积过滤，去掉零碎噪声，同时保留主要发光区域。
        cv::Mat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(mask_red_glow, labels, stats, centroids, 8, CV_32S);
        cv::Mat mask_filtered = cv::Mat::zeros(mask_red_glow.size(), CV_8UC1);
        int kept_blobs = 0;
        for (int i = 1; i < num_labels; ++i) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area >= area_min) {
                cv::Mat one = (labels == i);
                mask_filtered.setTo(255, one);
                kept_blobs++;
            }
        }
        mask_red_glow = mask_filtered;

        cv::Mat red_only;
        cv::bitwise_and(proc_frame, proc_frame, red_only, mask_red_glow);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask_red_glow, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool has_square = false;
        cv::Rect best_square;
        double best_score = -1.0;
        double best_area = 0.0;

        for (const auto &c : contours) {
            const double area = cv::contourArea(c);
            if (area < sq_area_min || area > sq_area_max) {
                continue;
            }
            cv::Rect rct = cv::boundingRect(c);
            if (rct.width <= 0 || rct.height <= 0) {
                continue;
            }

            const double ar = static_cast<double>(rct.width) / static_cast<double>(rct.height);
            const double ar_dev = std::abs(ar - 1.0);
            const double ar_tol = static_cast<double>(sq_ar_tol_percent) / 100.0;
            if (ar_dev > ar_tol) {
                continue;
            }

            // 更偏好接近正方形且面积更大的候选。
            const double square_score = (1.0 - ar_dev / ar_tol) * area;
            if (square_score > best_score) {
                best_score = square_score;
                best_square = rct;
                best_area = area;
                has_square = true;
            }
        }

        if (has_square) {
            cv::rectangle(red_only, best_square, cv::Scalar(0, 255, 255), 2);
            cv::putText(red_only,
                        cv::format("Square A=%.0f", best_area),
                        cv::Point(best_square.x, std::max(20, best_square.y - 8)),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.65,
                        cv::Scalar(0, 255, 255),
                        2);
        }

        cv::Mat red_show;
        if (red_only.size() != frame.size()) {
            cv::resize(red_only, red_show, frame.size(), 0, 0, cv::INTER_NEAREST);
        } else {
            red_show = red_only;
        }

        cv::Rect square_on_raw;
        if (has_square) {
            if (proc_scale < 100) {
                const double inv_s = 100.0 / static_cast<double>(proc_scale);
                square_on_raw.x = static_cast<int>(std::round(best_square.x * inv_s));
                square_on_raw.y = static_cast<int>(std::round(best_square.y * inv_s));
                square_on_raw.width = static_cast<int>(std::round(best_square.width * inv_s));
                square_on_raw.height = static_cast<int>(std::round(best_square.height * inv_s));
            } else {
                square_on_raw = best_square;
            }
            square_on_raw &= cv::Rect(0, 0, frame.cols, frame.rows);
        }

        cv::Mat frame_show = frame.clone();

        auto now = std::chrono::steady_clock::now();
        if (!fps_init_) {
            last_frame_time_ = now;
            fps_init_ = true;
        }
        const double dt_ms = std::chrono::duration<double, std::milli>(now - last_frame_time_).count();
        last_frame_time_ = now;
        const double fps = (dt_ms > 0.0) ? (1000.0 / dt_ms) : 0.0;

        cv::putText(frame_show,
                    cv::format("FPS: %.1f", fps),
                    cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(0, 255, 255),
                    2);
        cv::putText(frame_show,
                    cv::format("Frames: %zu", total_frame_count_ + 1),
                    cv::Point(20, 80),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(0, 255, 0),
                    2);
        cv::putText(frame_show,
                cv::format("Red pixels: %d", cv::countNonZero(mask_red_glow)),
                cv::Point(20, 120),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(0, 0, 255),
                2);
        cv::putText(frame_show,
                cv::format("Blobs: %d", kept_blobs),
                cv::Point(20, 160),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(0, 128, 255),
                2);

        cv::putText(frame_show,
                cv::format("H1[%d,%d] H2[%d,%d] S>=%d V>=%d Glow>=%d Bias=%d Area>=%d K=%d",
                       h1_low, h1_high, h2_low, h2_high,
                       s_low, v_low, v_glow_low, red_bias, area_min, kernel_size),
                cv::Point(20, 200),
                cv::FONT_HERSHEY_SIMPLEX,
                0.65,
                cv::Scalar(255, 255, 0),
                2);
        cv::putText(frame_show,
            cv::format("Process scale: %d%%", proc_scale),
            cv::Point(20, 240),
            cv::FONT_HERSHEY_SIMPLEX,
            0.8,
            cv::Scalar(255, 200, 0),
            2);
        cv::putText(frame_show,
            cv::format("Square[min=%d max=%d ar_tol=%.2f]", sq_area_min, sq_area_max, static_cast<double>(sq_ar_tol_percent) / 100.0),
            cv::Point(20, 280),
            cv::FONT_HERSHEY_SIMPLEX,
            0.8,
            cv::Scalar(0, 255, 255),
            2);

        if (has_square && square_on_raw.area() > 0) {
            cv::rectangle(frame_show, square_on_raw, cv::Scalar(0, 255, 255), 2);
            cv::putText(frame_show,
                "Target Square",
                cv::Point(square_on_raw.x, std::max(20, square_on_raw.y - 8)),
                cv::FONT_HERSHEY_SIMPLEX,
                0.65,
                cv::Scalar(0, 255, 255),
                2);
        }

        cv::putText(red_show,
                "Extracted Red Glow",
                cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(0, 0, 255),
                2);
        if (has_square) {
            cv::putText(red_show,
                "Square Found",
                cv::Point(20, 80),
                cv::FONT_HERSHEY_SIMPLEX,
                0.9,
                cv::Scalar(0, 255, 255),
                2);
        } else {
            cv::putText(red_show,
                "Square Not Found",
                cv::Point(20, 80),
                cv::FONT_HERSHEY_SIMPLEX,
                0.9,
                cv::Scalar(0, 165, 255),
                2);
        }
        cv::imshow("DetectDartRaw", frame_show);
        cv::imshow("DetectDartExtract", red_show);
        cv::waitKey(1);

        ++frame_count_;
        ++total_frame_count_;
    }

    void print_stats() {
        RCLCPP_INFO(get_logger(),
                    "[DetectDart] received compressed frames in last 5s: %zu, total: %zu",
                    frame_count_,
                    total_frame_count_);
        frame_count_ = 0;
    }

    bool fps_init_ = false;
    std::chrono::steady_clock::time_point last_frame_time_{};
    int h1_low_ = 0;
    int h1_high_ = 15;
    int h2_low_ = 160;
    int h2_high_ = 179;
    int s_low_ = 70;
    int v_low_ = 70;
    int v_glow_low_ = 100;
    int red_bias_ = 15;
    int area_min_ = 20;
    int morph_size_ = 3;
    int proc_scale_ = 60;
    int square_area_min_ = 30;
    int square_area_max_ = 1500;
    int square_ar_tol_percent_ = 35;
    size_t frame_count_ = 0;
    size_t total_frame_count_ = 0;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_;
    rclcpp::TimerBase::SharedPtr stat_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectDart>());
    rclcpp::shutdown();
    return 0;
}
