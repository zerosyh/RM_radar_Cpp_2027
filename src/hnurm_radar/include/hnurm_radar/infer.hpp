#pragma once
#include <cuda_runtime.h>
#include <NvInfer.h>
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include "hnurm_radar/tensorrt_inference.hpp"

struct InferArmor {
    cv::Rect abs_rect;          // 原图绝对坐标
    std::string label;
    float conf;
    float field_x = 0.0f;
    float field_y = 0.0f;
    bool valid_field = false;
    float car_bottom_x = 0.0f;  // 车辆底部中心在原图上的x
    float car_bottom_y = 0.0f;  // 车辆底部中心在原图上的y
};

class InferEngine {
public:
    InferEngine(const std::string& stage1_path,
                const std::string& stage2_path,
                const std::string& stage3_path,
                float conf_car, float conf_armor,
                const std::vector<std::string>& labels);
    ~InferEngine();

    std::vector<InferArmor> infer(const cv::Mat& frame);

    // 获取最近一次推理的车辆框（1280x1280坐标）
    const std::vector<cv::Rect>& getLastCarBoxes() const { return car_boxes_; }

    // 获取最近一次推理的内部耗时（毫秒）
    float getLastCarTime() const { return last_car_time_; }
    float getLastArmorTime() const { return last_armor_time_; }
    float getLastTotalTime() const { return last_total_time_; }

    InferEngine(const InferEngine&) = delete;
    InferEngine& operator=(const InferEngine&) = delete;

private:
    nvinfer1::IRuntime* runtime_;
    EngineInfo engine_car_;
    EngineInfo engine_armor_;
    EngineInfo engine_digit_;

    cudaStream_t stream_;
    uint8_t *d_scratch_car_;
    uint8_t *d_scratch_armor_;

    float conf_car_;
    float conf_armor_;
    std::vector<std::string> labels_;

    // 临时缓存
    std::vector<cv::Rect> car_boxes_;
    std::vector<std::pair<float,float>> car_bottom_pts_;
    std::vector<std::vector<float>> armor_host_buffers_;

    // 内部耗时记录
    float last_car_time_ = 0.0f;
    float last_armor_time_ = 0.0f;
    float last_total_time_ = 0.0f;
};