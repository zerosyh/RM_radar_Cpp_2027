#pragma once
#include <opencv2/opencv.hpp>
#include <cmath>
#include <array>
#include <vector>

// 场地代价地图 — 从 cost_map.png 加载，28×15 像素，1px=1m
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

    float at(float fx, float fy) const {
        int x = (int)(fx + 0.5f), y = (int)(fy + 0.5f);
        if (x < 0 || x >= W || y < 0 || y >= H) return COST_WALL;
        return grid[x][y];
    }

    // A*: 最小代价路径长度 + 返回路径点
    float pathLength(float sx, float sy, float gx, float gy,
                     std::vector<std::pair<int,int>>* out_path = nullptr) const {
        int si = clampX(sx), sj = clampY(sy);
        int gi = clampX(gx), gj = clampY(gy);
        if (out_path) out_path->clear();
        if (si == gi && sj == gj) { if (out_path) out_path->push_back({si,sj}); return 0; }

        bool closed[W][H]{};
        float g[W][H];
        int parent[W][H][2];
        for (int x = 0; x < W; ++x) for (int y = 0; y < H; ++y)
            { g[x][y] = 1e9f; parent[x][y][0] = -1; }

        struct Node { int x, y; float f; };
        std::vector<Node> open;
        g[si][sj] = 0;
        open.push_back({si, sj, heur(si, sj, gi, gj)});

        const int DX[4] = {1, -1, 0, 0}, DY[4] = {0, 0, 1, -1};

        while (!open.empty()) {
            size_t best = 0;
            for (size_t i = 1; i < open.size(); ++i)
                if (open[i].f < open[best].f) best = i;
            Node cur = open[best];
            open[best] = open.back(); open.pop_back();
            if (cur.x == gi && cur.y == gj) {
                if (out_path) {
                    int cx = gi, cy = gj;
                    while (cx >= 0) {
                        out_path->push_back({cx, cy});
                        int px = parent[cx][cy][0], py = parent[cx][cy][1];
                        cx = px; cy = py;
                    }
                    std::reverse(out_path->begin(), out_path->end());
                }
                return g[gi][gj];
            }
            if (closed[cur.x][cur.y]) continue;
            closed[cur.x][cur.y] = true;

            for (int d = 0; d < 4; ++d) {
                int nx = cur.x + DX[d], ny = cur.y + DY[d];
                if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                if (grid[nx][ny] >= COST_WALL) continue;
                if (closed[nx][ny]) continue;
                float ng = g[cur.x][cur.y] + grid[nx][ny];
                if (ng < g[nx][ny]) {
                    g[nx][ny] = ng;
                    parent[nx][ny][0] = cur.x;
                    parent[nx][ny][1] = cur.y;
                    open.push_back({nx, ny, ng + heur(nx, ny, gi, gj)});
                }
            }
        }
        return 1e9f;
    }

    // 取路径上距起点 dist_m 的导航点
    void navPoint(float sx, float sy, float gx, float gy, float dist_m,
                  float& nx, float& ny) const {
        float dx = gx - sx, dy = gy - sy;
        float d = std::hypot(dx, dy);
        if (d < 0.1f) { nx = gx; ny = gy; return; }
        nx = sx + (dx / d) * dist_m;
        ny = sy + (dy / d) * dist_m;
        for (int t = 0; t < 5 && at(nx, ny) >= COST_WALL; ++t) {
            float p = 0.5f * (t + 1);
            if (at(nx + p, ny) < COST_WALL) { nx += p; break; }
            if (at(nx - p, ny) < COST_WALL) { nx -= p; break; }
            if (at(nx, ny + p) < COST_WALL) { ny += p; break; }
            if (at(nx, ny - p) < COST_WALL) { ny -= p; break; }
        }
        nx = std::clamp(nx, 1.0f, (W-1)*1.0f);
        ny = std::clamp(ny, 1.0f, (H-1)*1.0f);
    }

private:
    int clampX(float x) const { return std::clamp((int)(x + 0.5f), 0, W-1); }
    int clampY(float y) const { return std::clamp((int)(y + 0.5f), 0, H-1); }
    float heur(int x, int y, int gx, int gy) const {
        return std::abs(x - gx) + std::abs(y - gy);
    }
};
