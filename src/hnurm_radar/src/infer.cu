#include "hnurm_radar/infer.hpp"
#include "hnurm_radar/preprocess.cuh"
#include <cuda_runtime.h>
#include <NvInfer.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>

// ==================== 预处理 kernel 实现 ====================
__global__ void preprocess_kernel(
    const uint8_t* __restrict__ src, int srcW, int srcH,
    float* __restrict__ dst, int dstW, int dstH,
    float scale, int dw, int dh, int new_w, int new_h,
    float norm = 1.0f / 255.0f
) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= dstW || y >= dstH) return;

    if (x < dw || x >= dw + new_w || y < dh || y >= dh + new_h) {
        float gray = 114.0f * norm;
        for (int c = 0; c < 3; ++c)
            dst[c * dstH * dstW + y * dstW + x] = gray;
        return;
    }

    float src_x = (x - dw) / scale;
    float src_y = (y - dh) / scale;

    int x0 = floorf(src_x), y0 = floorf(src_y);
    int x1 = x0 + 1, y1 = y0 + 1;
    x0 = max(0, min(x0, srcW - 1));
    x1 = max(0, min(x1, srcW - 1));
    y0 = max(0, min(y0, srcH - 1));
    y1 = max(0, min(y1, srcH - 1));

    float wx1 = src_x - x0, wy1 = src_y - y0;
    float wx0 = 1.0f - wx1, wy0 = 1.0f - wy1;

    const uint8_t* p00 = src + (y0 * srcW + x0) * 3;
    const uint8_t* p01 = src + (y0 * srcW + x1) * 3;
    const uint8_t* p10 = src + (y1 * srcW + x0) * 3;
    const uint8_t* p11 = src + (y1 * srcW + x1) * 3;

    for (int c = 0; c < 3; ++c) {
        float val = (p00[c] * wx0 * wy0 + p01[c] * wx1 * wy0 +
                     p10[c] * wx0 * wy1 + p11[c] * wx1 * wy1) * norm;
        dst[c * dstH * dstW + y * dstW + x] = val;
    }
}

__global__ void preprocess_classify_kernel(
    const uint8_t* __restrict__ src, int srcW, int srcH,
    float* __restrict__ dst, int dstW, int dstH,
    float meanR, float meanG, float meanB,
    float stdR, float stdG, float stdB
) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= dstW || y >= dstH) return;

    float src_x = (x + 0.5f) * (srcW / (float)dstW) - 0.5f;
    float src_y = (y + 0.5f) * (srcH / (float)dstH) - 0.5f;

    int x0 = floorf(src_x), y0 = floorf(src_y);
    int x1 = x0 + 1, y1 = y0 + 1;
    x0 = max(0, min(x0, srcW - 1));
    x1 = max(0, min(x1, srcW - 1));
    y0 = max(0, min(y0, srcH - 1));
    y1 = max(0, min(y1, srcH - 1));

    float wx1 = src_x - x0, wy1 = src_y - y0;
    float wx0 = 1.0f - wx1, wy0 = 1.0f - wy1;

    const uint8_t* p00 = src + (y0 * srcW + x0) * 3;
    const uint8_t* p01 = src + (y0 * srcW + x1) * 3;
    const uint8_t* p10 = src + (y1 * srcW + x0) * 3;
    const uint8_t* p11 = src + (y1 * srcW + x1) * 3;

    float R = (p00[0] * wx0 * wy0 + p01[0] * wx1 * wy0 +
               p10[0] * wx0 * wy1 + p11[0] * wx1 * wy1) / 255.0f;
    float G = (p00[1] * wx0 * wy0 + p01[1] * wx1 * wy0 +
               p10[1] * wx0 * wy1 + p11[1] * wx1 * wy1) / 255.0f;
    float B = (p00[2] * wx0 * wy0 + p01[2] * wx1 * wy0 +
               p10[2] * wx0 * wy1 + p11[2] * wx1 * wy1) / 255.0f;

    dst[0 * dstH * dstW + y * dstW + x] = (R - meanR) / stdR;
    dst[1 * dstH * dstW + y * dstW + x] = (G - meanG) / stdG;
    dst[2 * dstH * dstW + y * dstW + x] = (B - meanB) / stdB;
}

// ==================== 预处理包装函数 ====================
void preprocess_on_gpu(const cv::Mat& cpu_frame, float* d_input, int inW, int inH, cudaStream_t stream) {
    int origW = cpu_frame.cols, origH = cpu_frame.rows;
    size_t srcSize = origW * origH * 3 * sizeof(uint8_t);
    uint8_t* d_src = nullptr;
    cudaMallocAsync(&d_src, srcSize, stream);
    cudaMemcpyAsync(d_src, cpu_frame.data, srcSize, cudaMemcpyHostToDevice, stream);

    float scale = std::min(static_cast<float>(inW) / origW, static_cast<float>(inH) / origH);
    int new_w = static_cast<int>(origW * scale);
    int new_h = static_cast<int>(origH * scale);
    int dw = (inW - new_w) / 2;
    int dh = (inH - new_h) / 2;

    dim3 block(32, 16);
    dim3 grid((inW + block.x - 1) / block.x, (inH + block.y - 1) / block.y);
    preprocess_kernel<<<grid, block, 0, stream>>>(d_src, origW, origH, d_input, inW, inH, scale, dw, dh, new_w, new_h);
    cudaFreeAsync(d_src, stream);
}

void preprocess_classify_on_gpu(const cv::Mat& cpu_frame, float* d_input, int inW, int inH, cudaStream_t stream) {
    int origW = cpu_frame.cols, origH = cpu_frame.rows;
    size_t srcSize = origW * origH * 3 * sizeof(uint8_t);
    uint8_t* d_src = nullptr;
    cudaMallocAsync(&d_src, srcSize, stream);
    cudaMemcpyAsync(d_src, cpu_frame.data, srcSize, cudaMemcpyHostToDevice, stream);

    dim3 block(32, 16);
    dim3 grid((inW + block.x - 1) / block.x, (inH + block.y - 1) / block.y);
    preprocess_classify_kernel<<<grid, block, 0, stream>>>(d_src, origW, origH, d_input, inW, inH,
                                                           0.485f, 0.456f, 0.406f,
                                                           0.229f, 0.224f, 0.225f);
    cudaFreeAsync(d_src, stream);
}

void preprocess_on_gpu_ex(const cv::Mat& cpu_frame, float* d_input, int inW, int inH,
                          cudaStream_t stream, uint8_t* d_scratch) {
    int origW = cpu_frame.cols, origH = cpu_frame.rows;
    size_t srcSize = origW * origH * 3 * sizeof(uint8_t);
    cudaMemcpyAsync(d_scratch, cpu_frame.data, srcSize, cudaMemcpyHostToDevice, stream);

    float scale = std::min(static_cast<float>(inW) / origW, static_cast<float>(inH) / origH);
    int new_w = static_cast<int>(origW * scale);
    int new_h = static_cast<int>(origH * scale);
    int dw = (inW - new_w) / 2;
    int dh = (inH - new_h) / 2;

    dim3 block(32, 16);
    dim3 grid((inW + block.x - 1) / block.x, (inH + block.y - 1) / block.y);
    preprocess_kernel<<<grid, block, 0, stream>>>(d_scratch, origW, origH, d_input, inW, inH, scale, dw, dh, new_w, new_h);
}

void preprocess_classify_on_gpu_ex(const cv::Mat& cpu_frame, float* d_input, int inW, int inH,
                                   cudaStream_t stream, uint8_t* d_scratch) {
    int origW = cpu_frame.cols, origH = cpu_frame.rows;
    size_t srcSize = origW * origH * 3 * sizeof(uint8_t);
    cudaMemcpyAsync(d_scratch, cpu_frame.data, srcSize, cudaMemcpyHostToDevice, stream);

    dim3 block(32, 16);
    dim3 grid((inW + block.x - 1) / block.x, (inH + block.y - 1) / block.y);
    preprocess_classify_kernel<<<grid, block, 0, stream>>>(d_scratch, origW, origH, d_input, inW, inH,
                                                           0.485f, 0.456f, 0.406f,
                                                           0.229f, 0.224f, 0.225f);
}

// ==================== InferEngine 实现 ====================
InferEngine::InferEngine(const std::string& stage1_path,
                         const std::string& stage2_path,
                         const std::string& stage3_path,
                         float conf_car, float conf_armor,
                         const std::vector<std::string>& labels)
    : conf_car_(conf_car), conf_armor_(conf_armor), labels_(labels) {
    runtime_ = nvinfer1::createInferRuntime(gLogger);
    engine_car_   = createEngineInfo(runtime_, stage1_path);
    engine_armor_ = createEngineInfo(runtime_, stage2_path);
    engine_digit_ = createEngineInfo(runtime_, stage3_path);

    cudaStreamCreate(&stream_);
    cudaMalloc(&d_scratch_car_, 1280 * 1280 * 3);
    cudaMalloc(&d_scratch_armor_, 192 * 192 * 3);
}

InferEngine::~InferEngine() {
    if (d_scratch_car_)  cudaFree(d_scratch_car_);
    if (d_scratch_armor_) cudaFree(d_scratch_armor_);
    if (stream_) cudaStreamDestroy(stream_);
    delete runtime_;
}

std::vector<InferArmor> InferEngine::infer(const cv::Mat& frame) {
    auto t_total_start = std::chrono::steady_clock::now();

    int orig_w = frame.cols, orig_h = frame.rows;
    cv::Mat frame_resized;
    cv::resize(frame, frame_resized, cv::Size(1280, 1280));
    float scale_x = static_cast<float>(orig_w) / 1280.0f;
    float scale_y = static_cast<float>(orig_h) / 1280.0f;

    // ========= 阶段1：整车检测 =========
    auto t_car_pre_start = std::chrono::steady_clock::now();
    preprocess_on_gpu_ex(frame_resized, static_cast<float*>(engine_car_.d_input), 1280, 1280, stream_, d_scratch_car_);
    auto t_car_pre_end = std::chrono::steady_clock::now();

    auto t_car_infer_start = std::chrono::steady_clock::now();
    engine_car_.context->enqueueV3(stream_);
    cudaMemcpyAsync(engine_car_.hostOutput.data(), engine_car_.d_output, engine_car_.outputSize, cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);
    auto t_car_infer_end = std::chrono::steady_clock::now();

    // 后处理：解析车辆框
    car_boxes_.clear();
    car_bottom_pts_.clear();
    for (int i = 0; i < engine_car_.outDim1; ++i) {
        float* det = engine_car_.hostOutput.data() + i * engine_car_.outDim2;
        float conf = det[4];
        if (conf < conf_car_) continue;
        float x1 = std::clamp(det[0], 0.f, 1279.f);
        float y1 = std::clamp(det[1], 0.f, 1279.f);
        float x2 = std::clamp(det[2], 0.f, 1279.f);
        float y2 = std::clamp(det[3], 0.f, 1279.f);
        car_boxes_.emplace_back(cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)));
        car_bottom_pts_.emplace_back((x1 + x2) / 2.0f, y2);
    }
    auto t_car_post_end = std::chrono::steady_clock::now();

    // 记录阶段1耗时
    last_car_preprocess_time_  = std::chrono::duration<float, std::milli>(t_car_pre_end - t_car_pre_start).count();
    last_car_infer_time_       = std::chrono::duration<float, std::milli>(t_car_infer_end - t_car_infer_start).count();
    last_car_postprocess_time_ = std::chrono::duration<float, std::milli>(t_car_post_end - t_car_infer_end).count();

    // ========= 阶段2+3：装甲板检测 + 数字分类 =========
    auto t_armor_pre_start = std::chrono::steady_clock::now();
    std::vector<InferArmor> armors;
    armor_host_buffers_.resize(car_boxes_.size(),
        std::vector<float>(engine_armor_.outDim1 * engine_armor_.outDim2));

    // 批量装甲板预处理 + 推理
    for (size_t i = 0; i < car_boxes_.size(); ++i) {
        cv::Rect car_exp = expand_bbox(car_boxes_[i], 1.2f, 1280, 1280);
        cv::Mat car_roi = frame_resized(car_exp);
        cv::Mat car_roi_rgb;
        cv::cvtColor(car_roi, car_roi_rgb, cv::COLOR_BGR2RGB);
        preprocess_on_gpu_ex(car_roi_rgb, static_cast<float*>(engine_armor_.d_input), 192, 192, stream_, d_scratch_car_);
        engine_armor_.context->enqueueV3(stream_);
        cudaMemcpyAsync(armor_host_buffers_[i].data(), engine_armor_.d_output, engine_armor_.outputSize, cudaMemcpyDeviceToHost, stream_);
    }
    cudaStreamSynchronize(stream_);
    auto t_armor_pre_end = std::chrono::steady_clock::now();  // 包含预处理+推理+同步

    // 解析装甲板 + 数字分类
    float digit_total_time = 0.0f;
    for (size_t i = 0; i < car_boxes_.size(); ++i) {
        cv::Rect car_exp = expand_bbox(car_boxes_[i], 1.2f, 1280, 1280);
        cv::Mat car_roi = frame_resized(car_exp);
        float scale3 = std::min(192.0f / car_roi.cols, 192.0f / car_roi.rows);
        int nw3 = car_roi.cols * scale3, nh3 = car_roi.rows * scale3;
        int dw3 = (192 - nw3) / 2, dh3 = (192 - nh3) / 2;
        auto& armor_out = armor_host_buffers_[i];

        float car_bottom_x_1280 = car_bottom_pts_[i].first;
        float car_bottom_y_1280 = car_bottom_pts_[i].second;

        for (int j = 0; j < engine_armor_.outDim1; ++j) {
            float* det = armor_out.data() + j * engine_armor_.outDim2;
            float conf = det[4];
            int cls = det[5];   // Stage2颜色类别：0=dead, 1=红, 2=蓝
            if (conf < conf_armor_) continue;

            float car_x1 = (det[0] - dw3) / scale3;
            float car_y1 = (det[1] - dh3) / scale3;
            float car_x2 = (det[2] - dw3) / scale3;
            float car_y2 = (det[3] - dh3) / scale3;
            cv::Rect a_rect_1280(std::clamp(int(car_x1),0,car_roi.cols-1),
                                 std::clamp(int(car_y1),0,car_roi.rows-1),
                                 std::clamp(int(car_x2-car_x1),0,car_roi.cols-1),
                                 std::clamp(int(car_y2-car_y1),0,car_roi.rows-1));
            if (a_rect_1280.width <= 0 || a_rect_1280.height <= 0) continue;

            // 数字分类（Stage3）
            cv::Rect a_exp = expand_bbox(a_rect_1280, 1.1f, car_roi.cols, car_roi.rows);
            cv::Mat armor_roi = car_roi(a_exp);
            cv::Mat armor_roi_rgb;
            cv::cvtColor(armor_roi, armor_roi_rgb, cv::COLOR_BGR2RGB);

            size_t classify_scratch_size = armor_roi_rgb.total() * 3;
            uint8_t* d_classify_scratch = nullptr;
            cudaMallocAsync(&d_classify_scratch, classify_scratch_size, stream_);

            auto t_digit_start = std::chrono::steady_clock::now();
            // 数字分类模型输入尺寸为 64x64
            preprocess_classify_on_gpu_ex(armor_roi_rgb, static_cast<float*>(engine_digit_.d_input), 64, 64, stream_, d_classify_scratch);
            engine_digit_.context->enqueueV3(stream_);
            cudaMemcpyAsync(engine_digit_.hostOutput.data(), engine_digit_.d_output, engine_digit_.outputSize, cudaMemcpyDeviceToHost, stream_);
            cudaStreamSynchronize(stream_);
            auto t_digit_end = std::chrono::steady_clock::now();
            digit_total_time += std::chrono::duration<float, std::milli>(t_digit_end - t_digit_start).count();

            cudaFreeAsync(d_classify_scratch, stream_);

            // 概率计算
            std::vector<float> probs(engine_digit_.hostOutput.begin(), engine_digit_.hostOutput.begin() + engine_digit_.outDim1);
            softmax(probs.data(), engine_digit_.outDim1);
            int max_idx = std::max_element(probs.begin(), probs.end()) - probs.begin();
            if (max_idx < 0 || max_idx >= static_cast<int>(labels_.size())) continue;

            // Stage2 颜色映射: 0->D, 1->R, 2->B
            char color_stage2;
            if (cls == 0)      color_stage2 = 'D';
            else if (cls == 1) color_stage2 = 'R';
            else if (cls == 2) color_stage2 = 'B';
            else continue;

            // 如需忽略 dead 装甲板，可取消下一行注释
            // if (color_stage2 == 'D') continue;

            // 最终标签 = 颜色 + 数字字符串
            std::string digit_str = labels_[max_idx];   // e.g. "1","2","S"
            std::string final_label = color_stage2 + digit_str;

            InferArmor ar;
            ar.abs_rect = cv::Rect(static_cast<int>((car_exp.x + a_rect_1280.x) * scale_x),
                                   static_cast<int>((car_exp.y + a_rect_1280.y) * scale_y),
                                   static_cast<int>(a_rect_1280.width * scale_x),
                                   static_cast<int>(a_rect_1280.height * scale_y));
            ar.label = final_label;
            ar.conf = probs[max_idx];
            ar.car_bottom_x = car_bottom_x_1280 * scale_x;
            ar.car_bottom_y = car_bottom_y_1280 * scale_y;
            armors.push_back(ar);
        }
    }
    auto t_armor_post_end = std::chrono::steady_clock::now();

    // 记录阶段2+3耗时
    last_armor_preprocess_time_  = std::chrono::duration<float, std::milli>(t_armor_pre_end - t_armor_pre_start).count();
    last_armor_infer_time_       = 0.0f;  // 已包含在 preprocess 中（异步流水线无法精确拆分）
    last_armor_postprocess_time_ = std::chrono::duration<float, std::milli>(t_armor_post_end - t_armor_pre_end).count() - digit_total_time;
    last_digit_total_time_       = digit_total_time;

    auto t_total_end = std::chrono::steady_clock::now();
    last_total_time_ = std::chrono::duration<float, std::milli>(t_total_end - t_total_start).count();

    return armors;
}
