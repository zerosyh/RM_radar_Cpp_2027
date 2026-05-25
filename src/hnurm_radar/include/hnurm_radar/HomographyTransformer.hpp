#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <utility>

class HomographyTransformer {
public:
    HomographyTransformer(const std::string& calib_path, const std::string& mask_path);
    std::pair<float, float> pixelToField(float px, float py, const std::string& my_color);

private:
    cv::Mat H_ground_, H_highland_, mask_img_;
    int map_w_ = 0, map_h_ = 0;
    bool map_portrait_ = false;
    std::pair<float, float> mapToField(float mx, float my, const std::string& my_color);
};