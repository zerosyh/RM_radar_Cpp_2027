#include <rclcpp/rclcpp.hpp>
#include <detect_result/msg/location.hpp>
#include <detect_result/msg/locations.hpp>
#include <yaml-cpp/yaml.h>
#include <hnurm_radar/cost_map.h>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;

enum Zone { Z_ATK_DEEP=0, Z_ATK_MID=1, Z_CENTER=2, Z_DEF_MID=3, Z_DEF_DEEP=4, Z_COUNT=5 };

class SentryDecisionNode : public rclcpp::Node {
public:
    SentryDecisionNode() : Node("sentry_decision") {
        auto cfg = YAML::LoadFile("configs/main_config.yaml");
        my_color_ = cfg["global"]["my_color"].as<std::string>("Red");
        sentry_id_ = (my_color_ == "Red") ? 7 : 107;
        if (cfg["sentry"]) {
            sx_ = cfg["sentry"]["x"].as<float>(1.0f); sy_ = cfg["sentry"]["y"].as<float>(7.5f);
        } else {
            sx_ = (my_color_ == "Red") ? 1.0f : 27.0f; sy_ = 7.5f;
        }
        tw_[1]=4; tw_[2]=2; tw_[3]=3; tw_[4]=3; tw_[7]=1;

        rclcpp::QoS q(rclcpp::KeepLast(5)); q.best_effort();
        sub_ = create_subscription<detect_result::msg::Locations>(
            "location", q, std::bind(&SentryDecisionNode::cb, this, std::placeholders::_1));
        rclcpp::QoS qp(rclcpp::KeepLast(5)); qp.reliable();
        pub_ = create_publisher<detect_result::msg::Locations>("sentry_targets", qp);

        if (!cm_.load("configs/cost_map.png")) cm_.loadDefault();
        RCLCPP_INFO(get_logger(), "SentryDecision ready. color=%s", my_color_.c_str());
    }

private:
    std::string my_color_; int sentry_id_; CostMap cm_;
    float sx_, sy_, psx_=0, psy_=0, svx_=0, svy_=0;
    std::chrono::steady_clock::time_point s_t_, last_pub_;
    static constexpr double PUB_I = 0.5;
    std::unordered_map<int,float> tw_;
    float lnx_=-999, lny_=-999;
    static constexpr float NH = 0.5f;

    rclcpp::Subscription<detect_result::msg::Locations>::SharedPtr sub_;
    rclcpp::Publisher<detect_result::msg::Locations>::SharedPtr pub_;

    Zone zx(float x) const {
        bool b = (my_color_ == "Blue");
        if (b) {
            if (x>24) return Z_DEF_DEEP; else if (x>18) return Z_DEF_MID;
            else if (x>10) return Z_CENTER; else if (x>3) return Z_ATK_MID;
            else return Z_ATK_DEEP;
        } else {
            if (x<4) return Z_DEF_DEEP; else if (x<10) return Z_DEF_MID;
            else if (x<18) return Z_CENTER; else if (x<25) return Z_ATK_MID;
            else return Z_ATK_DEEP;
        }
    }

    void cb(const detect_result::msg::Locations::SharedPtr msg) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_pub_).count() < PUB_I) return;
        last_pub_ = now;

        // 哨兵实时位置
        for (auto& l : msg->locs) {
            if (l.id == sentry_id_) {
                psx_=sx_; psy_=sy_; sx_=l.x; sy_=l.y;
                if (s_t_.time_since_epoch().count()>0) {
                    float dt=std::max(0.1f,(float)std::chrono::duration<double>(now-s_t_).count());
                    svx_=(sx_-psx_)/dt; svy_=(sy_-psy_)/dt;
                }
                s_t_=now; break;
            }
        }

        // 分区统计 (敌方+我方, 排除无人机)
        float zt_en[Z_COUNT]={}; int zc_en[Z_COUNT]={};
        float zt_fr[Z_COUNT]={}; int zc_fr[Z_COUNT]={};
        int total_en=0, total_fr=0;
        for (auto& l : msg->locs) {
            if (l.id<0||l.id>=600||l.id==6||l.id==106) continue;
            bool en = (my_color_=="Red") ? (l.id>=100) : (l.id<100&&l.id>0);
            bool fr = !en && (l.id>0) && (l.id!=sentry_id_);
            int n=l.id%100; float w=tw_.count(n)?tw_[n]:1.0f;
            if (en) {
                zt_en[zx(l.x)]+=w; zc_en[zx(l.x)]++; total_en++;
            } else if (fr) {
                zt_fr[zx(l.x)]+=w; zc_fr[zx(l.x)]++; total_fr++;
            }
        }

        auto lm = std::make_unique<detect_result::msg::Locations>();

        // SENTRY
        detect_result::msg::Location sn;
        sn.x=sx_; sn.y=sy_; sn.z=0; sn.id=901; sn.label="SENTRY"; lm->locs.push_back(sn);

        // 警戒方向
        float ax=0,ay=0,aw=0;
        for (auto& l : msg->locs) {
            if (l.id<0||l.id>=600||l.id==6||l.id==106) continue;
            bool en = (my_color_=="Red") ? (l.id>=100) : (l.id<100&&l.id>0);
            if (!en) continue;
            float dx=l.x-sx_, dy=l.y-sy_, d=std::hypot(dx,dy)+0.1f;
            float w=tw_.count(l.id%100)?tw_[l.id%100]:1.0f;
            ax+=(dx/d)*w; ay+=(dy/d)*w; aw+=w;
        }
        if (aw>0) {
            detect_result::msg::Location al;
            al.x=sx_+(ax/aw)*8; al.y=sy_+(ay/aw)*8; al.z=0;
            al.id=900; al.label="ALERT"; lm->locs.push_back(al);
        }

        if (total_en==0) {
            detect_result::msg::Location nv;
            nv.x=sx_; nv.y=sy_; nv.z=0; nv.id=800; nv.label="NAV"; lm->locs.push_back(nv);
            pub_->publish(std::move(lm)); return;
        }

        // ---- 攻防判定 (考虑我方兵力) ----
        int def_en = zc_en[Z_DEF_MID]+zc_en[Z_DEF_DEEP];
        int def_fr = zc_fr[Z_DEF_MID]+zc_fr[Z_DEF_DEEP];
        // 防守条件: 我方半场敌多我少 → 守; 否则攻
        bool is_def = (def_en > def_fr);

        // 敌方重心 (距离加权, 近处敌人权重更高)
        float cx=0, cy=0, cw=0;
        for (auto& l : msg->locs) {
            if (l.id<0||l.id>=600||l.id==6||l.id==106) continue;
            bool en=(my_color_=="Red")?(l.id>=100):(l.id<100&&l.id>0);
            if (!en) continue;
            float w=tw_.count(l.id%100)?tw_[l.id%100]:1.0f;
            float dist = std::hypot(l.x-sx_, l.y-sy_) + 0.5f;
            float dw = w / dist;  // 距离倒数加权
            cx+=l.x*dw; cy+=l.y*dw; cw+=dw;
        }
        cx/=cw; cy/=cw;

        // NAV = 重心 + 偏移(始终偏向哨兵侧3m) + 场地边界钳位
        float offset = (my_color_=="Blue") ? 3.0f : -3.0f;
        float raw_nx = std::clamp(cx + offset, 1.0f, 27.0f);
        float raw_ny = std::clamp(cy, 2.0f, 13.0f);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "[SENTRY_DBG] sx=%.1f sy=%.1f cx=%.1f cy=%.1f mode=%s nav=(%.1f,%.1f)",
            sx_, sy_, cx, cy, is_def?"DEF":"ATK", raw_nx, raw_ny);

        // 速度方向补偿
        float sdx=raw_nx-sx_, sdy=raw_ny-sy_, sd=std::hypot(sdx,sdy);
        if (sd>0.1f && std::abs(svx_)+std::abs(svy_)>0.2f) {
            float dot=(svx_*sdx+svy_*sdy)/sd;
            float fac=std::clamp(1.0f-dot*0.15f,0.5f,1.8f);
            raw_nx=sx_+(sdx/sd)*(sd*fac); raw_ny=sy_+(sdy/sd)*(sd*fac);
        }
        if (lnx_<-900 || std::hypot(raw_nx-lnx_,raw_ny-lny_)>NH) { lnx_=raw_nx; lny_=raw_ny; }

        // NAV
        detect_result::msg::Location nv;
        nv.x=lnx_; nv.y=lny_; nv.z=0; nv.id=801; nv.label=(is_def?"DEF":"ATK");
        lm->locs.push_back(nv);

        // 模式标签
        detect_result::msg::Location md;
        md.x=sx_; md.y=sy_+1; md.z=0; md.id=902; md.label=(is_def?"DEF":"ATK");
        lm->locs.push_back(md);

        pub_->publish(std::move(lm));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SentryDecisionNode>());
    rclcpp::shutdown();
    return 0;
}
