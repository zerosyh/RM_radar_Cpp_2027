#pragma once
#include <opencv2/opencv.hpp>
#include <array>

// 场地代价地图 — 从 cost_map.png 加载，56×30 网格，0.5m 分辨率
struct CostMap {
    static constexpr int W = 56, H = 30;  // 0.5m 分辨率
    static constexpr float COST_FLAT   = 1.0f;
    static constexpr float COST_SLOPE  = 2.5f;
    static constexpr float COST_WALL   = 999.0f;

    std::array<std::array<float, H>, W> grid{};

    bool load(const std::string& path) {
        cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
        if (img.empty() || img.cols != W || img.rows != H) return false;

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                cv::Vec3b p = img.at<cv::Vec3b>(y, x);
                int r = p[2], g = p[1], b = p[0];  // BGR→RGB

                if (r < 60 && g < 60 && b < 60)
                    grid[x][y] = COST_WALL;   // 黑色 → 障碍
                else if (r < 210 && g < 210 && b < 210)
                    grid[x][y] = COST_SLOPE;  // 灰色 → 坡道
                else
                    grid[x][y] = COST_FLAT;   // 白色 → 平地
            }
        }
        return true;
    }

    // 默认硬编码 (fallback)
    void loadDefault() {
        for (int x = 0; x < W; ++x)
            for (int y = 0; y < H; ++y)
                grid[x][y] = COST_FLAT;

        // 红蓝补给站 (坐标 x2)
        for (int x = 0; x < 6; ++x) for (int y = 0; y < 4; ++y) grid[x][y] = COST_WALL;
        for (int x = 50; x < W; ++x) for (int y = 26; y < H; ++y) grid[x][y] = COST_WALL;

        // 环高柱区
        for (int x = 34; x <= 38; ++x) for (int y = 12; y <= 16; ++y) grid[x][y] = COST_WALL;

        // 坡道
        for (int x = 32; x <= 42; ++x) {
            for (int y = 4; y <= 8; ++y)   if (grid[x][y] < COST_WALL) grid[x][y] = COST_SLOPE;
            for (int y = 20; y <= 24; ++y) if (grid[x][y] < COST_WALL) grid[x][y] = COST_SLOPE;
        }

        // 场地边界
        for (int x = 0; x < W; ++x) { grid[x][0] = COST_WALL; grid[x][H-1] = COST_WALL; }
        for (int y = 0; y < H; ++y) { grid[0][y] = COST_WALL; grid[W-1][y] = COST_WALL; }
    }
};
