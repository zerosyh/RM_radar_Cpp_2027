#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <detect_result/msg/location.hpp>
#include <detect_result/msg/locations.hpp>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class DisplayPanel : public rclcpp::Node {
public:
    DisplayPanel() : Node("display_panel") {
        RCLCPP_INFO(get_logger(), "Display Panel Node is running");

        auto main_cfg = YAML::LoadFile("configs/main_config.yaml");
        auto scene_name = main_cfg["global"]["scene"].as<std::string>("competition");
        auto scenes = main_cfg["scenes"];
        auto scene = scenes[scene_name];

        field_w_ = scene["field_width"].as<float>(28.0f);
        field_h_ = scene["field_height"].as<float>(15.0f);
        map_px_w_ = static_cast<int>(field_w_ * 100);
        map_px_h_ = static_cast<int>(field_h_ * 100);

        std::string std_map_rel = scene["std_map"].as<std::string>("source/maps/competition_2026/std_map.png");
        std::string map_path;
        if (std_map_rel[0] == '/') {
            map_path = std_map_rel;
        } else {
            auto root = std::string(std::getenv("HOME")) + "/rm_lidar_2027/RM_radar_Cpp_2027";
            map_path = root + "/" + std_map_rel;
        }

        map_img_ = cv::imread(map_path);
        if (map_img_.empty()) {
            RCLCPP_WARN(get_logger(), "无法加载地图: %s，使用空白图", map_path.c_str());
            map_img_ = cv::Mat(map_px_h_, map_px_w_, CV_8UC3, cv::Scalar(240, 240, 240));
        } else {
            cv::resize(map_img_, map_img_, cv::Size(map_px_w_, map_px_h_));
        }

        rclcpp::QoS qos(rclcpp::KeepLast(3));
        qos.best_effort();

        sub_location_ = create_subscription<detect_result::msg::Locations>(
            "location", qos,
            std::bind(&DisplayPanel::locationCb, this, std::placeholders::_1));

        rclcpp::QoS pub_qos(rclcpp::KeepLast(1));
        pub_qos.reliable();
        pub_map_view_ = create_publisher<sensor_msgs::msg::Image>("/map_view", pub_qos);

        cv::namedWindow("map", cv::WINDOW_NORMAL);
        cv::resizeWindow("map", 800, 471);

        display_thread_ = std::thread(&DisplayPanel::displayLoop, this);
    }

    ~DisplayPanel() {
        running_ = false;
        if (display_thread_.joinable()) display_thread_.join();
        cv::destroyAllWindows();
    }

private:
    void locationCb(const detect_result::msg::Locations::SharedPtr msg) {
        std::lock_guard<std::mutex> l(mtx_);
        latest_locs_ = *msg;
    }

    void drawRobotMarker(cv::Mat& img, int xx, int yy, const cv::Scalar& color, bool is_air, float z) {
        if (is_air) {
            cv::Scalar air_color(0, 255, 0);
            std::vector<cv::Point> pts = {
                {xx, yy - 50}, {xx + 50, yy}, {xx, yy + 50}, {xx - 50, yy}
            };
            cv::polylines(img, pts, true, air_color, 3);
            cv::circle(img, cv::Point(xx, yy), 5, air_color, -1);
            char buf[64];
            snprintf(buf, sizeof(buf), "h=%.1fm", z);
            cv::putText(img, buf, cv::Point(xx + 55, yy + 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
        } else {
            cv::circle(img, cv::Point(xx, yy), 60, color, 4);
        }
    }

    void displayLoop() {
        int pub_counter = 0;
        const int PUB_EVERY_N = 5;

        while (rclcpp::ok() && running_) {
            detect_result::msg::Locations locs_copy;
            {
                std::lock_guard<std::mutex> l(mtx_);
                locs_copy = latest_locs_;
            }

            cv::Mat show_map = map_img_.clone();

            for (const auto& loc : locs_copy.locs) {
                float x = loc.x;
                float y = loc.y;
                float z = loc.z;

                int xx = static_cast<int>(x * 100);
                int yy = map_px_h_ - static_cast<int>(y * 100);

                bool is_air = (loc.id == 6 || loc.id == 106 ||
                              (loc.id >= 600 && loc.id < 700) ||
                              (loc.id >= 1600 && loc.id < 1700));
                std::string air_suffix = is_air ? " UAV" : "";

                cv::Scalar color;
                if (loc.label == "Red") {
                    color = cv::Scalar(0, 0, 255);
                } else if (loc.label == "Blue") {
                    color = cv::Scalar(255, 0, 0);
                } else {
                    color = cv::Scalar(128, 128, 128);
                }

                char label_text[64];
                if (loc.label == "Red" || loc.label == "Blue") {
                    snprintf(label_text, sizeof(label_text), "%d%s", loc.id, air_suffix.c_str());
                } else {
                    snprintf(label_text, sizeof(label_text), "null");
                }

                char coord_text[32];
                snprintf(coord_text, sizeof(coord_text), "%.1f,%.1f", x, y);

                cv::putText(show_map, label_text, cv::Point(xx - 15, yy + 10),
                            cv::FONT_HERSHEY_SIMPLEX, 2, color, 4);
                cv::putText(show_map, coord_text, cv::Point(xx, yy - 60),
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 4);

                if (loc.label == "Red" || loc.label == "Blue") {
                    drawRobotMarker(show_map, xx, yy, color, is_air, z);
                } else {
                    cv::circle(show_map, cv::Point(xx, yy), 60, color, 4);
                }
            }

            cv::imshow("map", show_map);
            cv::waitKey(16);

            pub_counter++;
            if (pub_counter >= PUB_EVERY_N) {
                pub_counter = 0;
                try {
                    cv::Mat small;
                    cv::resize(show_map, small, cv::Size(show_map.cols / 2, show_map.rows / 2));
                    auto img_msg = cv_bridge::CvImage(
                        std_msgs::msg::Header(), "bgr8", small).toImageMsg();
                    img_msg->header.stamp = this->now();
                    img_msg->header.frame_id = "map";
                    pub_map_view_->publish(*img_msg);
                } catch (const std::exception& e) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                         "发布小地图失败: %s", e.what());
                }
            }

            std::this_thread::sleep_for(66ms);  // ~15fps
        }
    }

    // ---------- 成员变量 ----------
    float field_w_, field_h_;
    int map_px_w_, map_px_h_;
    cv::Mat map_img_;

    detect_result::msg::Locations latest_locs_;
    std::mutex mtx_;

    rclcpp::Subscription<detect_result::msg::Locations>::SharedPtr sub_location_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_map_view_;

    std::thread display_thread_;
    bool running_ = true;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisplayPanel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
