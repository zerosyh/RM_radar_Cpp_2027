#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <deque>
#include <mutex>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>

class PcdQueue{

    private:
        mutable std::mutex mtx_;
        size_t max_size_;
        std::deque<pcl::PointCloud<pcl::PointXYZ>> queue_;
        pcl::PointCloud<pcl::PointXYZ> all_;
        void rebuild(){
            all_.clear();
            for(const auto & pc:queue_) all_ +=pc;
        }
    public:
        explicit PcdQueue(size_t max_size=10):max_size_(max_size){}
        void add(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
        {
            std::lock_guard<std::mutex>lock(mtx_);
            queue_.push_back(*cloud);
            if(queue_.size()>max_size_)queue_.pop_front();
            rebuild();
        }

        pcl::PointCloud<pcl::PointXYZ> getAll() const{
            std::lock_guard<std::mutex>lock (mtx_);
            return all_;
        }
};

class DynamicCloudNode : public rclcpp::Node {

    private:
    //
    std::string lidar_frame_, map_frame_, lidar_topic_name_;
    //
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcds_, pub_targets_, pub_other_, pub_bg_debug_;
    //
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    //
    std::thread pub_thread_;
    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ>kdtree_;
    //
    PcdQueue pcd_queue_;
    std::deque<pcl::PointCloud<pcl::PointXYZ>> accumulated_clouds_;
    std::deque<pcl::PointCloud<pcl::PointXYZ>> other_accumulated_clouds_;
    std::mutex accum_mutex_;
    //
    double min_distance_, max_distance_;
    int accumulate_time_;
    double background_threshold_;

    //fun
    void loadMap(const std::string& path,float leaf_size){
        map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(path,*map_cloud_)==-1){
            RCLCPP_ERROR(this->get_logger(),"FAiled to load map%s",path.c_str());
            return;
        }
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(map_cloud_);
        vg.setLeafSize(leaf_size,leaf_size,leaf_size);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
        vg.filter(*filtered);
        map_cloud_=filtered;
        kdtree_.setInputCloud(map_cloud_);
    
    }   

    pcl::PointCloud<pcl::PointXYZ>::Ptr extractXYZ(const sensor_msgs::msg::PointCloud2::SharedPtr&msg){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg,*cloud);
        return cloud;
    }
    sensor_msgs::msg::PointCloud2::SharedPtr packCloud(const pcl::PointCloud<pcl::PointXYZ>&cloud,const std::string& frame_id)
    {
        auto msg = std ::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(cloud,*msg);
        msg->header.frame_id=frame_id;
        msg->header.stamp =this->now();
        return msg;
    }
    void publishRawLoop(){
        rclcpp::Rate rate(100);
        while(rclcpp::ok()){
            auto all_pc = pcd_queue_.getAll();
            if(!all_pc.empty()){
                pub_pcds_->publish(*packCloud(all_pc,lidar_frame_));
            }
            rate.sleep();
        }
    }
    // 飞镖区域（蓝方堡垒右上角引导灯附近）
    static bool isDartRegion(float x, float y, float z) {
        return (x > 28.0f - 0.5889f - 0.1885f && x < 28.0f - 0.5889f) &&
               (y > 3.925f && y < 4.525f) &&
               (z > 2.4722f - 0.859f + 0.1f && z < 2.4722f);
    }

    // ---- 蓝方无人机区域（低Y侧，停机坪上方） ----
    static bool isFlyBlue(float x, float y, float z) {
        return (x > 13.5f && x < 27.0f) &&
               (y > 0.5f && y < 4.5f) &&
               (z > 1.7f && z < 3.5f);
    }

    // ---- 红方无人机区域（高Y侧，停机坪上方） ----
    static bool isFlyRed(float x, float y, float z) {
        return (x > 2.0f && x < 14.5f) &&
               (y > 10.5f && y < 14.3f) &&
               (z > 1.7f && z < 3.5f);
    }
    //
    pcl::PointCloud<pcl::PointXYZ> extractDynamic(const pcl::PointCloud<pcl::PointXYZ>&cloud,double threshold)
    {
        pcl::PointCloud<pcl::PointXYZ> dynamic_pts;
        for(const auto&pt:cloud.points){
            std::vector<int>idx(1);
            std::vector<float>dist(1);
            if(kdtree_.nearestKSearch(pt,1,idx,dist)>0){
                if(std::sqrt(dist[0])>threshold){
                    dynamic_pts.push_back(pt);
                }
            }
        }
        return dynamic_pts;
    }
    //
    void detectAndLog(const pcl::PointCloud<pcl::PointXYZ>&other_points){
        if(other_points.empty()) return;
        int dart_cnt=0, fly_blue_cnt = 0, fly_red_cnt = 0;
        for (const auto& pt : other_points.points) {
            if (isDartRegion(pt.x, pt.y, pt.z)) dart_cnt++;
            if (isFlyBlue(pt.x, pt.y, pt.z))  fly_blue_cnt++;
            if (isFlyRed(pt.x, pt.y, pt.z))   fly_red_cnt++;
        }

        if (dart_cnt > 5)   RCLCPP_WARN(this->get_logger(), "发现飞镖！点数: %d", dart_cnt);
        if (fly_blue_cnt > 40) RCLCPP_WARN(this->get_logger(), "蓝方无人机区域检测到目标！点数: %d", fly_blue_cnt);
        if (fly_red_cnt  > 40) RCLCPP_WARN(this->get_logger(), "红方无人机区域检测到目标！点数: %d", fly_red_cnt);
    }
    
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        auto t_start=std::chrono::steady_clock::now();

        pcl::PointCloud<pcl::PointXYZ>::Ptr points_lidar=extractXYZ(msg);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl :: PointCloud<pcl::PointXYZ>());
        for (const auto&pt : points_lidar->points){
            double dist = std::sqrt (pt.x*pt.x+pt.y*pt.y+pt.z*pt.z);
            if(dist>min_distance_ && dist<max_distance_)
                filtered->push_back(pt);
        }
        if(!filtered->empty()){
            pcd_queue_.add(filtered);
        }
        geometry_msgs::msg::TransformStamped transform;
        try{
            transform=tf_buffer_->lookupTransform(map_frame_,msg->header.frame_id,tf2::TimePointZero,tf2::durationFromSec(0.1));

        }catch(const tf2::TransformException&ex)
        {
            RCLCPP_WARN(this->get_logger(),"TF error：%s",ex.what());
            return;
        }

        Eigen::Affine3d T_lidar2map = tf2::transformToEigen(transform);
        Eigen::Affine3d T_map2lidar = T_lidar2map.inverse();
        Eigen::Affine3f T_lidar2map_f= T_lidar2map.cast<float>();
        Eigen::Affine3f T_map2lidar_f= T_map2lidar.cast<float>();
        pcl::PointCloud<pcl::PointXYZ> points_map;
        // 防止 filtered 为空时调用 transform 导致崩溃
        if (!filtered->empty()) {
            pcl::transformPointCloud(*filtered,points_map,T_lidar2map_f);
        }
        
        pcl::PointCloud<pcl::PointXYZ> filtered_main, other;
        for (const auto& pt : points_map.points) {
            bool main_cond = (pt.x < 3) || (pt.x > 28) || (pt.y < 0) || (pt.y > 15) ||
                             (pt.z < 0) || (pt.z > 1.4) ||
                             ((pt.y > 0) && (pt.y < 5) && (pt.x > 25)) ||
                             ((21.5 - 2.9/sqrt(2)) < (pt.x + pt.y) && (pt.x + pt.y) < (21.5 + 2.9/sqrt(2)) &&
                              (-6.5 - 0.9/sqrt(2)) < (pt.y - pt.x) && (pt.y - pt.x) < (-6.5 + 0.9/sqrt(2)));

            bool other_flag = isDartRegion(pt.x, pt.y, pt.z) ||
                              isFlyBlue(pt.x, pt.y, pt.z) ||
                              isFlyRed(pt.x, pt.y, pt.z);

            if (main_cond) {
                if (other_flag) other.push_back(pt);
                continue;
            }
            filtered_main.push_back(pt);
        }


        pcl::PointCloud<pcl::PointXYZ> dynamic_map=extractDynamic(filtered_main,background_threshold_);
        pcl::PointCloud<pcl::PointXYZ> dynamic_lidar,other_lidar;
        // 关键修复：变换前检查点云是否为空，避免 width=0 导致 PCL 内部除零
        if (!dynamic_map.empty()) {
            pcl::transformPointCloud(dynamic_map,dynamic_lidar,T_map2lidar_f);
        }
        if (!other.empty()) {
            pcl::transformPointCloud(other,other_lidar,T_map2lidar_f);
        }
        {
            std::lock_guard<std::mutex>lock(accum_mutex_);
            if (!dynamic_lidar.empty()) {
                accumulated_clouds_.push_back(dynamic_lidar);
                if (accumulated_clouds_.size() > static_cast<size_t>(accumulate_time_))
                    accumulated_clouds_.pop_front();
            }
            if (!other_lidar.empty()) {
                other_accumulated_clouds_.push_back(other_lidar);
                if (other_accumulated_clouds_.size() > static_cast<size_t>(accumulate_time_))
                    other_accumulated_clouds_.pop_front();
            }
        }
        pcl::PointCloud<pcl::PointXYZ> acc_dynamic, acc_other;
        {
            std::lock_guard<std::mutex> lock(accum_mutex_);
            for (const auto& c : accumulated_clouds_)       acc_dynamic += c;
            for (const auto& c : other_accumulated_clouds_) acc_other += c;
        }

        if (!acc_dynamic.empty()) pub_targets_->publish(*packCloud(acc_dynamic, lidar_frame_));
        if (!acc_other.empty())   pub_other_->publish(*packCloud(acc_other, lidar_frame_));

        detectAndLog(other);

        if (pub_bg_debug_->get_subscription_count() > 0) {
            pub_bg_debug_->publish(*packCloud(*map_cloud_, map_frame_));
        }

        auto t_end = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        RCLCPP_DEBUG(this->get_logger(), "Callback time: %.2f ms", dt);
        
    }
    public:
    //
    DynamicCloudNode():Node("dynamic_cloud_node"),pcd_queue_(10)
    {
        std::string config_path="configs/main_config.yaml";
        YAML::Node cfg=YAML::LoadFile(config_path);
        auto lidar_cfg=cfg["lidar"];
        min_distance_=lidar_cfg["min_distance"].as<double>();
        max_distance_=lidar_cfg["max_distance"].as<double>();
        lidar_topic_name_=lidar_cfg["lidar_topic_name"].as<std::string>();
        auto camera_cfg=cfg["camera"];
        std::string camera_mode=camera_cfg["mode"].as<std::string>();
        if (camera_mode == "rosbag") {
            lidar_frame_ = camera_cfg["tf_source_frame"].as<std::string>("livox_frame");
            map_frame_    = camera_cfg["tf_target_frame"].as<std::string>("map");
        } else {
            lidar_frame_ = "livox_frame";
            map_frame_   = "map";
        }
        accumulate_time_     = 3;
        background_threshold_ = 0.20;
        std::string map_path = "/home/syh/rm_lidar_2027/HNURM-radar-2026/data/pointclouds/background/RM2025.pcd";
        loadMap(map_path, 0.1f);
        RCLCPP_INFO(this->get_logger(), "Static map loaded: %zu points", map_cloud_->size());

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

         rclcpp::QoS qos(10);
        qos.reliable();

        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_name_, qos,
            std::bind(&DynamicCloudNode::callback, this, std::placeholders::_1));

        pub_pcds_    = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_pcds", qos);
        pub_targets_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("target_pointcloud", qos);
        pub_other_   = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/lidar_other", qos);
        pub_bg_debug_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("background_map_debug", qos);

        pub_thread_ = std::thread(&DynamicCloudNode::publishRawLoop, this);
    }
    ~DynamicCloudNode() {
        if (pub_thread_.joinable()) pub_thread_.join();
    }
};


int main(int argc,char**argv){  
    rclcpp::init(argc,argv);
    auto node = std::make_shared<DynamicCloudNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
