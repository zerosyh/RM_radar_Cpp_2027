#include <rclcpp/rclcpp.hpp>
#include <detect_result/msg/location.hpp>
#include <detect_result/msg/locations.hpp>
#include <yaml-cpp/yaml.h>
#include <NvInfer.h>
#include <cuda_runtime.h>
#include <deque>
#include <fstream>
#include <vector>
#include <cmath>
#include <mutex>
#include <atomic>

using namespace std::chrono_literals;

// TRT logger
class TRTLogger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            RCLCPP_WARN(rclcpp::get_logger("ai_decision"), "[TRT] %s", msg);
    }
};
static TRTLogger gLogger;

// ============================================================
// ai_decision: 哨兵 BC 模型接入节点 (初步)
// 输入:  location (雷达融合, 敌方位置 + 哨兵位置)
// 状态:  101d 哨兵状态 (位置-only + 时刻条件均值填充)
// 推理:  哨兵 BC 模型 v2 (TRT engine, seq=128 滑窗)
// 输出:  ai_nav (Locations, id=801 NAV waypoint 绝对坐标)
// ============================================================

// 场地几何 (裁判系, 与 robomaster_ai building_pos.json 一致)
static constexpr float FIELD_W = 28.0f, FIELD_H = 15.0f;
static constexpr float BUILDING_X[4] = {2.44f, 10.87f, 17.12f, 25.51f};  // 红基/红前哨/蓝前哨/蓝基
static constexpr float BUILDING_Y[4] = {7.44f, 3.63f, 11.38f, 7.61f};
// 装配点 (数据反推) — 本节点暂不用于状态
static constexpr int SEQ_LEN = 128;
static constexpr int STATE_DIM = 101;
static constexpr int OUT_DIM = 12;

class TensorRTEngine {
public:
    bool load(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) { RCLCPP_ERROR(rclcpp::get_logger("ai_decision"), "engine 加载失败: %s", path.c_str()); return false; }
        std::vector<char> data((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        runtime_ = nvinfer1::createInferRuntime(gLogger);
        engine_ = runtime_->deserializeCudaEngine(data.data(), data.size());
        if (!engine_) return false;
        ctx_ = engine_->createExecutionContext();
        // 输入 [1,128,101] float32, 输出 [1,12] float32
        cudaMalloc(&d_in_, SEQ_LEN * STATE_DIM * sizeof(float));
        cudaMalloc(&d_out_, OUT_DIM * sizeof(float));
        h_out_.resize(OUT_DIM);
        return true;
    }
    void infer(const float* h_in, float* h_out) {
        cudaMemcpy(d_in_, h_in, SEQ_LEN * STATE_DIM * sizeof(float), cudaMemcpyHostToDevice);
        ctx_->setTensorAddress("states", d_in_);
        ctx_->setTensorAddress("pred", d_out_);
        ctx_->enqueueV3(0);
        cudaStreamSynchronize(0);
        cudaMemcpy(h_out_.data(), d_out_, OUT_DIM * sizeof(float), cudaMemcpyDeviceToHost);
        memcpy(h_out, h_out_.data(), OUT_DIM * sizeof(float));
    }
    ~TensorRTEngine() {
        if (d_in_) cudaFree(d_in_);
        if (d_out_) cudaFree(d_out_);
        if (ctx_) delete ctx_;
        if (engine_) delete engine_;
        if (runtime_) delete runtime_;
    }
private:
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* ctx_ = nullptr;
    void* d_in_ = nullptr; void* d_out_ = nullptr;
    std::vector<float> h_out_;
};

class AIDecisionNode : public rclcpp::Node {
public:
    AIDecisionNode() : Node("ai_decision") {
        // 配置
        auto cfg = YAML::LoadFile("configs/main_config.yaml");
        my_color_ = cfg["global"]["my_color"].as<std::string>("Red");
        std::string engine_path = cfg["ai"]["engine_path"].as<std::string>(
            "model/ONNX/sentry_v2.engine");
        std::string means_path = cfg["ai"]["means_path"].as<std::string>(
            "model/ONNX/sentry_state_means.bin");

        // 加载 TRT engine
        if (!engine_.load(engine_path)) {
            RCLCPP_ERROR(get_logger(), "TRT engine 加载失败, 节点禁用推理");
            enabled_ = false;
        }
        // 加载时刻均值矩阵 (419, 101)
        loadMeans(means_path);

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliable();
        sub_loc_ = create_subscription<detect_result::msg::Locations>(
            "location", qos, std::bind(&AIDecisionNode::locCb, this, std::placeholders::_1));
        pub_nav_ = create_publisher<detect_result::msg::Locations>("ai_nav", qos);
        RCLCPP_INFO(get_logger(), "AIDecisionNode OK. color=%s enabled=%d", my_color_.c_str(), (int)enabled_);
    }

private:
    std::string my_color_;
    bool enabled_ = true;
    TensorRTEngine engine_;
    std::vector<float> means_;   // 419*101
    int means_T_ = 0;

    std::deque<std::vector<float>> hist_;   // 128 帧滑窗
    std::mutex mtx_;

    void loadMeans(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) { RCLCPP_WARN(get_logger(), "均值矩阵加载失败: %s", path.c_str()); return; }
        int32_t hdr[2];
        f.read(reinterpret_cast<char*>(hdr), 8);
        means_T_ = hdr[0];
        means_.resize(means_T_ * STATE_DIM);
        f.read(reinterpret_cast<char*>(means_.data()), means_.size() * sizeof(float));
        RCLCPP_INFO(get_logger(), "均值矩阵: %d x %d", means_T_, STATE_DIM);
    }

    float meanAt(int t, int dim) const {
        int idx = std::min(std::max(t, 0), means_T_ - 1);
        return means_[idx * STATE_DIM + dim];
    }

    void locCb(const detect_result::msg::Locations::SharedPtr msg) {
        if (!enabled_) return;
        std::lock_guard<std::mutex> lock(mtx_);
        RCLCPP_INFO(get_logger(), "[locCb] 收到 %zu 个位置", msg->locs.size());

        // 1. 提取: 哨兵自身位置 + 敌方位置
        bool is_red = (my_color_ == "Red");
        int self_id = is_red ? 7 : 107;
        bool self_found = false;
        float self_x = 0, self_y = 0;
        std::vector<std::pair<float, float>> enemies;  // 场地坐标 (未镜像)

        for (const auto& loc : msg->locs) {
            if (loc.id < 0) continue;  // last_known 不用于状态 (待定: 可后续接入)
            if (loc.id == self_id) { self_found = true; self_x = loc.x; self_y = loc.y; continue; }
            bool is_enemy = is_red ? (loc.id >= 100) : (loc.id < 100 && loc.id > 0);
            if (is_enemy) enemies.emplace_back(loc.x, loc.y);
        }
        if (!self_found) { RCLCPP_INFO(get_logger(), "[locCb] 无哨兵位置, 跳过"); return; }
        RCLCPP_INFO(get_logger(), "[locCb] 哨兵(%.1f,%.1f) 敌方 %zu 台", self_x, self_y, enemies.size());

        // 2. 构造 101d 状态 (红方统一视角: 蓝方镜像 x→28-x)
        float mx = is_red ? 0.0f : FIELD_W;
        std::vector<float> sv(STATE_DIM);
        int t = (int)(std::min(hist_.size(), (size_t)means_T_ - 1));  // 时刻=帧数

        // self 17d: x,y 真实 (镜像), 其余时刻均值
        sv[0] = (is_red ? self_x : FIELD_W - self_x) / FIELD_W;
        sv[1] = self_y / FIELD_H;
        for (int i = 2; i < 16; ++i) sv[i] = meanAt(t, i);
        sv[16] = t / 420.0f;

        // ally 24d: 位置真实 (从 location 提取己方 hero/engineer/inf3/inf4), hp 用均值
        std::vector<float> ally_slots(24, 0.0f);
        std::vector<std::pair<float, float>> allies;
        std::vector<int> ally_ids = is_red ? std::vector<int>{1, 2, 3, 4} : std::vector<int>{101, 102, 103, 104};
        for (const auto& loc : msg->locs) {
            if (loc.id < 0) continue;
            for (int aid : ally_ids)
                if (loc.id == aid) { allies.emplace_back(loc.x, loc.y); break; }
        }
        for (size_t k = 0; k < allies.size() && k < 4; ++k) {
            float ax = (is_red ? allies[k].first : FIELD_W - allies[k].first);
            float ay = allies[k].second;
            float dx = ax - sv[0] * FIELD_W, dy = ay - sv[1] * FIELD_H;
            int base = (int)k * 6;
            ally_slots[base + 0] = 1.0f;
            ally_slots[base + 1] = std::clamp(dx / 10.0f, -3.0f, 3.0f);
            ally_slots[base + 2] = std::clamp(dy / 10.0f, -3.0f, 3.0f);
            ally_slots[base + 3] = meanAt(t, 17 + base + 3);   // hp 均值 (0x0003 未接入)
            ally_slots[base + 4] = std::min(std::hypot(dx, dy) / 28.0f, 1.0f);
            ally_slots[base + 5] = 0.0f;
        }
        for (int i = 17; i < 41; ++i) sv[i] = ally_slots[i - 17];

        // enemy 36d: 6 槽 (exist, dx, dy, hp, dist, marked) — 位置真实, hp/marked 均值
        std::vector<float> en_slots(36, 0.0f);
        for (size_t k = 0; k < enemies.size() && k < 6; ++k) {
            float ex = (is_red ? enemies[k].first : FIELD_W - enemies[k].first);
            float ey = enemies[k].second;
            float dx = ex - sv[0] * FIELD_W, dy = ey - sv[1] * FIELD_H;
            int base = (int)k * 6;   // en_slots 自身索引 0-35 (勿加 41!)
            en_slots[base + 0] = 1.0f;
            en_slots[base + 1] = std::clamp(dx / 10.0f, -3.0f, 3.0f);
            en_slots[base + 2] = std::clamp(dy / 10.0f, -3.0f, 3.0f);
            en_slots[base + 3] = meanAt(t, 41 + base + 3);  // hp 均值 (sv 偏移 41+)
            en_slots[base + 4] = std::min(std::hypot(dx, dy) / 28.0f, 1.0f);
            en_slots[base + 5] = 0.0f;                       // marked 0
        }
        for (int i = 41; i < 77; ++i) sv[i] = en_slots[i - 41];

        // zones 3d (77-79): 场地几何 (敌方建筑距离, 归一化)
        float my_enemy_outpost_x = is_red ? BUILDING_X[1] : FIELD_W - BUILDING_X[1];
        float my_enemy_outpost_y = BUILDING_Y[1];
        float my_enemy_base_x = is_red ? BUILDING_X[3] : FIELD_W - BUILDING_X[3];
        float my_enemy_base_y = BUILDING_Y[3];
        sv[77] = std::min(std::hypot(sv[0]*FIELD_W - my_enemy_outpost_x, sv[1]*FIELD_H - my_enemy_outpost_y) / 30.0f, 1.0f);
        sv[78] = std::min(std::hypot(sv[0]*FIELD_W - my_enemy_base_x, sv[1]*FIELD_H - my_enemy_base_y) / 30.0f, 1.0f);
        sv[79] = std::min(std::hypot(sv[0]*FIELD_W - 14.0f, sv[1]*FIELD_H - 7.5f) / 15.0f, 1.0f);

        // sentry 6d (80-85): 时刻均值
        for (int i = 80; i < 86; ++i) sv[i] = meanAt(t, i);

        // global 15d (86-100): 建筑距离可算, HP/金币均值, 计数从位置算
        sv[86] = meanAt(t, 86); sv[87] = meanAt(t, 87);
        sv[88] = meanAt(t, 88); sv[89] = meanAt(t, 89);
        sv[90] = meanAt(t, 90); sv[91] = meanAt(t, 91);
        float deo = std::hypot(sv[0]*FIELD_W - my_enemy_outpost_x, sv[1]*FIELD_H - my_enemy_outpost_y);
        float deb = std::hypot(sv[0]*FIELD_W - my_enemy_base_x, sv[1]*FIELD_H - my_enemy_base_y);
        sv[92] = std::min(deo / 30.0f, 1.0f);
        sv[93] = std::min(deb / 30.0f, 1.0f);
        sv[94] = std::clamp((my_enemy_outpost_x - sv[0]*FIELD_W) / 15.0f, -1.0f, 1.0f);
        sv[95] = std::clamp((my_enemy_outpost_y - sv[1]*FIELD_H) / 15.0f, -1.0f, 1.0f);
        sv[96] = meanAt(t, 96);  // gold 均值
        sv[97] = std::min((float)enemies.size() / 6.0f, 1.0f);
        sv[98] = 0.0f;  // allies_near: 无己方信息 → 0
        float nearest = 30.0f;
        for (auto& e : enemies) {
            float d = std::hypot(e.first - self_x, e.second - self_y);
            if (d < nearest) nearest = d;
        }
        sv[99] = std::min(nearest / 30.0f, 1.0f);
        sv[100] = std::min(std::hypot(sv[0]*FIELD_W, sv[1]*FIELD_H - 7.5f) / 30.0f, 1.0f);  // supply 距离 (红方补给 0,7.5; 蓝方 28,7.5 镜像后同)

        // 3. 滑窗推理
        hist_.push_back(sv);
        if (hist_.size() > SEQ_LEN) hist_.pop_front();
        if (hist_.size() < 16) return;  // 至少 16 帧

        std::vector<float> in(SEQ_LEN * STATE_DIM, 0.0f);
        int n = (int)hist_.size();
        int offset = SEQ_LEN - n;
        for (int i = 0; i < n; ++i)
            memcpy(&in[(offset + i) * STATE_DIM], hist_[i].data(), STATE_DIM * sizeof(float));

        float out[OUT_DIM];
        engine_.infer(in.data(), out);

        // 4. 输出: waypoint (wx, wy) 绝对坐标, 蓝方镜像回来
        float wx = out[0], wy = out[1];
        float nav_x = is_red ? wx * FIELD_W : FIELD_W - wx * FIELD_W;
        float nav_y = wy * FIELD_H;
        nav_x = std::clamp(nav_x, 0.0f, FIELD_W);
        nav_y = std::clamp(nav_y, 0.0f, FIELD_H);

        detect_result::msg::Locations nav;
        detect_result::msg::Location loc;
        loc.x = nav_x; loc.y = nav_y; loc.z = 0.0f;
        loc.id = 801; loc.label = "NAV";
        nav.locs.push_back(loc);
        pub_nav_->publish(nav);
    }

    rclcpp::Subscription<detect_result::msg::Locations>::SharedPtr sub_loc_;
    rclcpp::Publisher<detect_result::msg::Locations>::SharedPtr pub_nav_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AIDecisionNode>());
    rclcpp::shutdown();
    return 0;
}
