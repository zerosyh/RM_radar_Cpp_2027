#include "hnurm_radar/HomographyTransformer.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <opencv2/imgcodecs.hpp>   // 添加：cv::imread 所在头文件
#include <algorithm>               // 添加：std::clamp
HomographyTransformer::HomographyTransformer(const std::string& calib_path, const std::string& mask_path) {
    std::ifstream f(calib_path);
    if (!f.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("Homography"), "无法打开标定文件: %s", calib_path.c_str());
        return;
    }
    auto data = nlohmann::json::parse(f);
    auto load_mat = [&](const std::string& key) -> cv::Mat {
        if (data.contains(key)) {
            cv::Mat m(3, 3, CV_64F);
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    m.at<double>(i, j) = data[key][i][j].get<double>();
            return m;
        }
        return cv::Mat();
    };
    H_ground_ = load_mat("H_ground");
    if (H_ground_.empty()) H_ground_ = load_mat("H");
    H_highland_ = load_mat("H_highland");
    if (data.contains("map_w") && data.contains("map_h")) {
        map_w_ = data["map_w"].get<int>();
        map_h_ = data["map_h"].get<int>();
        map_portrait_ = data.value("map_is_portrait", false);
    }
    if (!mask_path.empty()) mask_img_ = cv::imread(mask_path, cv::IMREAD_COLOR);
}

std::pair<float, float> HomographyTransformer::mapToField(float mx, float my, const std::string& my_color) {
    if (map_w_ == 0) return {mx, my};
    float fx, fy;
    if (map_portrait_) {
        if (my_color == "Red") {
            fx = (map_h_ - my) / map_h_ * 28.0f;
            fy = (map_w_ - mx) / map_w_ * 15.0f;
        } else {
            fx = my / map_h_ * 28.0f;
            fy = mx / map_w_ * 15.0f;
        }
    } else {
        fx = mx / map_w_ * 28.0f;
        fy = (map_h_ - my) / map_h_ * 15.0f;
    }
    return {fx, fy};
}

std::pair<float, float> HomographyTransformer::pixelToField(float px, float py, const std::string& my_color) {
    if (H_ground_.empty()) return {-1, -1};
    std::vector<cv::Point2f> src{cv::Point2f(px, py)};
    std::vector<cv::Point2f> dst;
    cv::perspectiveTransform(src, dst, H_ground_);
    float mx = dst[0].x, my = dst[0].y;
    if (map_w_ > 0 && (mx < -500 || mx > map_w_ + 500 || my < -500 || my > map_h_ + 500))
        return {-1, -1};
    if (!mask_img_.empty() && !H_highland_.empty()) {
        int imx = std::clamp(int(mx * mask_img_.cols / map_w_), 0, mask_img_.cols - 1);
        int imy = std::clamp(int(my * mask_img_.rows / map_h_), 0, mask_img_.rows - 1);
        auto pixel = mask_img_.at<cv::Vec3b>(imy, imx);
        if (pixel != cv::Vec3b(0,0,0)) {
            cv::perspectiveTransform(src, dst, H_highland_);
            mx = dst[0].x; my = dst[0].y;
        }
    }
    return mapToField(mx, my, my_color);
}