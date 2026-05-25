#ifndef TENSORRT_INFERENCE_HPP_
#define TENSORRT_INFERENCE_HPP_

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include "hnurm_radar/preprocess.cuh"


// ========== 全局 Logger（与之前一致） ==========
class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cout << "[TRT] " << msg << std::endl;
    }
};

inline Logger gLogger;  // 全局实例，可在多个引擎间共享

// ========== 读取 engine 文件 ==========
inline std::vector<char> loadEngine(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.good()) throw std::runtime_error("无法打开 engine 文件: " + path);
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> data(size);
    file.read(data.data(), size);
    return data;
}

// ========== 引擎信息结构 ==========
struct EngineInfo {
    nvinfer1::ICudaEngine* engine = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
    std::string inputName, outputName;
    int inputC, inputH, inputW;
    size_t inputSize;
    int outDim1, outDim2;
    size_t outputSize;
    void* d_input = nullptr;
    void* d_output = nullptr;
    std::vector<float> hostOutput;

    // 分配 GPU 内存并绑定
    void allocGPU() {
        cudaMalloc(&d_input, inputSize);
        cudaMalloc(&d_output, outputSize);
        context->setInputTensorAddress(inputName.c_str(), d_input);
        context->setOutputTensorAddress(outputName.c_str(), d_output);
    }

    // 释放 GPU 内存
    void freeGPU() {
        if (d_input)  cudaFree(d_input);
        if (d_output) cudaFree(d_output);
    }
};

// ========== 创建 EngineInfo（自动加载 engine 并查询 IO 维度） ==========
inline EngineInfo createEngineInfo(nvinfer1::IRuntime* runtime, const std::string& path) {
    EngineInfo info;
    auto data = loadEngine(path);
    info.engine = runtime->deserializeCudaEngine(data.data(), data.size());
    if (!info.engine) throw std::runtime_error("反序列化 engine 失败: " + path);
    info.context = info.engine->createExecutionContext();
    if (!info.context) throw std::runtime_error("创建执行上下文失败: " + path);

    // 遍历所有 IO 张量，获取输入 / 输出名称和形状
    for (int i = 0; i < info.engine->getNbIOTensors(); ++i) {
        const char* name = info.engine->getIOTensorName(i);
        auto mode = info.engine->getTensorIOMode(name);
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            info.inputName = name;
            auto dims = info.engine->getTensorShape(name);
            info.inputC = dims.d[1];
            info.inputH = dims.d[2];
            info.inputW = dims.d[3];
            info.inputSize = dims.d[0] * info.inputC * info.inputH * info.inputW * sizeof(float);
        } else {
            info.outputName = name;
            auto dims = info.engine->getTensorShape(name);
            if (dims.nbDims == 2) {
                // 处理形状 [1, N] 或 [N, 1]
                if (dims.d[0] == 1 && dims.d[1] > 1) {
                    info.outDim1 = dims.d[1];
                    info.outDim2 = 1;
                } else {
                    info.outDim1 = dims.d[0];
                    info.outDim2 = dims.d[1];
                }
            } else if (dims.nbDims == 3) {
                info.outDim1 = dims.d[1];
                info.outDim2 = dims.d[2];
            }
            info.outputSize = info.outDim1 * info.outDim2 * sizeof(float);
        }
    }

    info.allocGPU();
    info.hostOutput.resize(info.outDim1 * info.outDim2);

    std::cout << "[Engine] " << path << " | input: " << info.inputC << "x" << info.inputH
              << "x" << info.inputW << " | output: " << info.outDim1 << "x" << info.outDim2 << std::endl;

    return info;
}

// ========== 边界框扩展 ==========
inline cv::Rect expand_bbox(const cv::Rect& box, float scale, int imgW, int imgH) {
    int cx = box.x + box.width / 2;
    int cy = box.y + box.height / 2;
    int bw = static_cast<int>(box.width * scale);
    int bh = static_cast<int>(box.height * scale);
    int x1 = std::max(0, cx - bw / 2);
    int y1 = std::max(0, cy - bh / 2);
    int x2 = std::min(imgW, cx + bw / 2);
    int y2 = std::min(imgH, cy + bh / 2);
    return cv::Rect(x1, y1, x2 - x1, y2 - y1);
}

// ========== Softmax ==========
inline void softmax(float* logits, int size) {
    float max_val = *std::max_element(logits, logits + size);
    float sum = 0.0f;
    for (int i = 0; i < size; ++i) {
        logits[i] = expf(logits[i] - max_val);
        sum += logits[i];
    }
    for (int i = 0; i < size; ++i) logits[i] /= sum;
}

#endif // TENSORRT_INFERENCE_HPP_