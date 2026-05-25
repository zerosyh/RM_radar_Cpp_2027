// ============================================================
// preprocess.cuh
// ============================================================
#pragma once
#include <cuda_runtime.h>
#include <opencv2/core.hpp>

// 原有接口（内部分配临时显存，适合单次调用）
void preprocess_on_gpu(
    const cv::Mat& cpu_frame,
    float* d_input, int inW, int inH,
    cudaStream_t stream = 0
);

void preprocess_classify_on_gpu(
    const cv::Mat& cpu_frame,
    float* d_input, int inW, int inH,
    cudaStream_t stream = 0
);

// 优化接口（使用外部传入的临时缓冲区，避免重复分配）
void preprocess_on_gpu_ex(
    const cv::Mat& cpu_frame,
    float* d_input, int inW, int inH,
    cudaStream_t stream,
    uint8_t* d_scratch
);

void preprocess_classify_on_gpu_ex(
    const cv::Mat& cpu_frame,
    float* d_input, int inW, int inH,
    cudaStream_t stream,
    uint8_t* d_scratch
);
