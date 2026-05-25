#pragma once
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <random>




#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp> 
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

//eigen libs
#include <Eigen/Geometry>
#include <Eigen/Core>


//small_gicp libs
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/benchmark/read_points.hpp>

//pcl libs
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

//teaserpp libs
#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <quatro/quatro_module.h>


#include <mutex>
namespace hnurm {

using QuatroPointType = pcl::PointXYZ; //can be changed


class RelocaliztionNode : public rclcpp::Node{
  public:
    // void run();
    explicit RelocaliztionNode(const rclcpp::NodeOptions &options);
    ~RelocaliztionNode()
    {
        RCLCPP_INFO(get_logger(), "RelocaliztionNode destroyed");
    }
    RelocaliztionNode(const RelocaliztionNode &)            = delete;
    RelocaliztionNode &operator=(const RelocaliztionNode &) = delete;
    RelocaliztionNode(RelocaliztionNode &&)                 = delete;
    RelocaliztionNode &operator=(RelocaliztionNode &&)      = delete;


  private://functions
    void pointcloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void load_pcd_map(const std::string& map_path);  
    void timer_callback();
    void timer_pub_tf_callback();

    void relocalization();
    void reset();
    void publishTransform(const Eigen::Matrix4d& transform_matrix);
    std::vector<Eigen::Isometry3d> generate_initial_guesses(const Eigen::Isometry3d& initial_pose, double trans_noise = 0.5,double rot_noise = M_PI/6);


  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_pub_tf_;


    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    

    //variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;  //target 
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_downsampled_;  //target 
    pcl::PointCloud<pcl::PointCovariance>::Ptr global_map_PointCovariance_;

    int accumulation_counter_ = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;  //source 
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulate_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_downsampled_;  //source 



    pcl::PointCloud<pcl::PointCovariance>::Ptr source_cloud_PointCovariance_;



    std::vector<Eigen::Isometry3d> guesses_;
    Eigen::Isometry3d initial_guess_ = Eigen::Isometry3d::Identity();

    geometry_msgs::msg::TransformStamped transform;


    struct RegistrationResult {
      Eigen::Isometry3d transformation;
      double error;
      bool converged;
   };
  


    // kd_tree
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;

    sensor_msgs::msg::PointCloud2::SharedPtr  cloud_;
    bool getInitialPose_ = false;
    bool doFirstRegistration_ = false;
    Eigen::Isometry3d pre_result_ = Eigen::Isometry3d::Identity();
    small_gicp::RegistrationResult result;

    bool get_first_tf_from_quatro_ = false;

    teaser::RobustRegistrationSolver::Params params;
    teaser::RobustRegistrationSolver solver(teaser::RobustRegistrationSolver::Params params);
    shared_ptr<quatro<QuatroPointType>> m_quatro_handler = nullptr;

    //params 
    std::string pointcloud_sub_topic_;
    std::string pcd_file_;
    std::string downsampled_pcd_file_;
    bool generate_downsampled_pcd_ = false;
    int num_threads_;
    int num_neighbors_;
    float max_dist_sq_;
    float source_voxel_size_;
    float map_voxel_size_;

    bool use_quatro_ = false;
    bool use_fixed_ = false;

    // 初始位姿参数（从 launch 注入，用于引导 ICP 收敛方向）
    double initial_pose_x_ = 0.0;
    double initial_pose_y_ = 0.0;
    double initial_pose_z_ = 0.0;
    double initial_pose_yaw_ = 0.0;
    bool use_config_initial_pose_ = false;  // 参数有效（不全为0）时为 true
    bool prefer_config_initial_pose_ = true;  // 为 true 时，优先用配置初始位姿启动 ICP，不先跑 Quatro
    double first_registration_max_translation_jump_ = 8.0;  // 首次 ICP 结果相对初值的最大允许平移跳变（米）
    bool using_config_initial_guess_ = false;  // 当前 initial_guess_ 是否来自配置

    //quatro++ params


    int m_rotation_max_iter_ = 100, m_num_max_corres_ = 200;
		double m_normal_radius_ = 0.02, m_fpfh_radius_ = 0.04, m_distance_threshold_ = 30.0;
		double m_noise_bound_ = 0.25, m_rotation_gnc_factor_ = 1.39, m_rotation_cost_thr_ = 0.0001;
		bool m_estimate_scale_ = false, m_use_optimized_matching_ = true;

    


    

  protected:
    std::shared_ptr<RelocaliztionNode> shared_from_this(){
      return std::static_pointer_cast<RelocaliztionNode>(rclcpp::Node::shared_from_this());
    }
};



}