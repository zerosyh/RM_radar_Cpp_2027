#include "registration/registration_node.hpp"
using namespace small_gicp;

namespace hnurm {

RelocaliztionNode::RelocaliztionNode(const rclcpp::NodeOptions &options)
        : Node("RelocaliztionNode", options) 
{
  RCLCPP_INFO(get_logger(), "RelocaliztionNode is running");
  // params declaration
  pointcloud_sub_topic_ = this->declare_parameter("pointcloud_sub_topic","/cloud_registered"); 
  pcd_file_ = this->declare_parameter("pcd_file","/home/rm/unit_test/ws/src/target.pcd");
  RCLCPP_INFO(get_logger(), "%s", pcd_file_.c_str());
  generate_downsampled_pcd_  = this->declare_parameter("generate_downsampled_pcd",false); 
  downsampled_pcd_file_ = this->declare_parameter("downsampled_pcd_file","/home/rm/unit_test/ws/src/target.pcd");
  RCLCPP_INFO(get_logger(), "%s", downsampled_pcd_file_.c_str());
  num_threads_ = this->declare_parameter("num_threads",4);
  num_neighbors_ = this->declare_parameter("num_neighbors",20);
  max_dist_sq_ = this->declare_parameter("max_dist_sq",1.0);
  source_voxel_size_ = this->declare_parameter("source_voxel_size",0.25);
  map_voxel_size_ = this->declare_parameter("map_voxel_size_",0.25);
  use_fixed_ = this->declare_parameter("use_fixed",false);
  use_quatro_ = this->declare_parameter("use_quatro",false);

  // 初始位姿参数（由 launch 文件从 main_config.yaml 注入）
  initial_pose_x_   = this->declare_parameter("initial_pose_x", 0.0);
  initial_pose_y_   = this->declare_parameter("initial_pose_y", 0.0);
  initial_pose_z_   = this->declare_parameter("initial_pose_z", 0.0);
  initial_pose_yaw_ = this->declare_parameter("initial_pose_yaw", 0.0);
  // 判断初始位姿是否有效（不全为0）
  use_config_initial_pose_ = !(initial_pose_x_ == 0.0 && initial_pose_y_ == 0.0
                                && initial_pose_z_ == 0.0 && initial_pose_yaw_ == 0.0);
  prefer_config_initial_pose_ = this->declare_parameter("prefer_config_initial_pose", true);
  first_registration_max_translation_jump_ =
    this->declare_parameter("first_registration_max_translation_jump", 8.0);
  if (use_config_initial_pose_) {
    RCLCPP_INFO(get_logger(),
      "Config initial pose: x=%.2f, y=%.2f, z=%.2f, yaw=%.4f",
      initial_pose_x_, initial_pose_y_, initial_pose_z_, initial_pose_yaw_);
    RCLCPP_INFO(get_logger(),
      "prefer_config_initial_pose=%s, first_registration_max_translation_jump=%.2f m",
      prefer_config_initial_pose_ ? "true" : "false",
      first_registration_max_translation_jump_);
  } else {
    RCLCPP_INFO(get_logger(),
      "No config initial pose set (all zeros), waiting for /initialpose or Quatro");
  }

  m_rotation_max_iter_ = this->declare_parameter("m_rotation_max_iter",100);
  m_num_max_corres_ = this->declare_parameter("m_num_max_corres",200);
  m_normal_radius_ = this->declare_parameter("m_normal_radius",0.02);
  m_fpfh_radius_ = this->declare_parameter("m_fpfh_radius",0.04);
  m_distance_threshold_ = this->declare_parameter("m_distance_threshold",30.0);
  m_noise_bound_ = this->declare_parameter("m_noise_bound",0.25);
  m_rotation_gnc_factor_ = this->declare_parameter("m_rotation_gnc_factor",1.39);
  m_rotation_cost_thr_ = this->declare_parameter("m_rotation_cost_thr",0.0001);
  m_estimate_scale_ = this->declare_parameter("m_estimate_scale",false);
  m_use_optimized_matching_ = this->declare_parameter("m_use_optimized_matching",true);

  if(!use_quatro_)
  {
    RCLCPP_INFO(get_logger(), "use small gicp mode,reading params......");
    RCLCPP_INFO(get_logger(), "get params: pointcloud_sub_topic =%s",pointcloud_sub_topic_.c_str());
    RCLCPP_INFO(get_logger(), "get params: pcd_file =%s",pcd_file_.c_str());
    RCLCPP_INFO(get_logger(), "get params: num_threads =%d",num_threads_);
    RCLCPP_INFO(get_logger(), "get params: num_neighbors =%d",num_neighbors_);
    RCLCPP_INFO(get_logger(), "get params: max_dist_sq =%f",max_dist_sq_);
    RCLCPP_INFO(get_logger(), "get params: source_voxel_size =%f",source_voxel_size_);
    RCLCPP_INFO(get_logger(), "get params: map_voxel_size =%f",map_voxel_size_);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "use quatro mode,reading params......");
    RCLCPP_INFO(get_logger(), "get params: m_rotation_max_iter =%d",m_rotation_max_iter_);
    RCLCPP_INFO(get_logger(), "get params: m_num_max_corres =%d",m_num_max_corres_);
    RCLCPP_INFO(get_logger(), "get params: m_normal_radius =%f",m_normal_radius_);
    RCLCPP_INFO(get_logger(), "get params: m_fpfh_radius =%f",m_fpfh_radius_);
    RCLCPP_INFO(get_logger(), "get params: m_distance_threshold =%f",m_distance_threshold_);
    RCLCPP_INFO(get_logger(), "get params: m_noise_bound =%f",m_noise_bound_);
    RCLCPP_INFO(get_logger(), "get params: m_rotation_gnc_factor =%f",m_rotation_gnc_factor_);
    RCLCPP_INFO(get_logger(), "get params: m_rotation_cost_thr =%f",m_rotation_cost_thr_);
    if(m_estimate_scale_) RCLCPP_INFO(get_logger(), "get params: m_estimate_scale = true");
    else RCLCPP_INFO(get_logger(), "get params: m_estimate_scale = false");
    if(m_use_optimized_matching_) RCLCPP_INFO(get_logger(), "get params: m_use_optimized_matching = true");
    else RCLCPP_INFO(get_logger(), "get params: m_use_optimized_matching = false");
  }
  
  //set quatro params && initialize handler
  m_quatro_handler = std::make_shared<quatro<QuatroPointType>>(m_normal_radius_, 
    m_fpfh_radius_, m_noise_bound_, m_rotation_gnc_factor_, m_rotation_cost_thr_,
    m_rotation_max_iter_, m_estimate_scale_, m_use_optimized_matching_, 
    m_distance_threshold_, m_num_max_corres_);
  
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // init pointcloud pointer
  source_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  accumulate_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  source_cloud_downsampled_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  global_map_downsampled_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);  
  global_map_PointCovariance_.reset(new pcl::PointCloud<pcl::PointCovariance>);

  //load pcd
  load_pcd_map(pcd_file_);
  rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
  qos_profile.reliable();

  //subscribers
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_sub_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&RelocaliztionNode::pointcloud_sub_callback,this,std::placeholders::_1)
  );

  init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    std::bind(&RelocaliztionNode::initial_pose_callback,this,std::placeholders::_1)
  );


  //publishers
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_pcd_map", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&RelocaliztionNode::timer_callback, this));
  timer_pub_tf_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&RelocaliztionNode::timer_pub_tf_callback, this));
  
}

void RelocaliztionNode::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if(use_fixed_&&get_first_tf_from_quatro_)  //enable reset initial pose
  {
    geometry_msgs::msg::Transform trans_;
  trans_.translation.x = msg->pose.pose.position.x;
  trans_.translation.y = msg->pose.pose.position.y;
  trans_.translation.z = msg->pose.pose.position.z;
  trans_.rotation.w = msg->pose.pose.orientation.w;
  trans_.rotation.x = msg->pose.pose.orientation.x;
  trans_.rotation.y = msg->pose.pose.orientation.y;
  trans_.rotation.z = msg->pose.pose.orientation.z;
  initial_guess_ = tf2::transformToEigen(trans_);
  // guesses_ = generate_initial_guesses(initial_guess_,0.5,M_PI/6);

  getInitialPose_ = true;
  using_config_initial_guess_ = false;
  RCLCPP_INFO(get_logger(), "Received initial pose:");
  RCLCPP_INFO(get_logger(), "  Position: [%.2f, %.2f, %.2f]", 
    msg->pose.pose.position.x, 
    msg->pose.pose.position.y, 
    msg->pose.pose.position.z);
  RCLCPP_INFO(get_logger(), "  Orientation: [%.2f, %.2f, %.2f, %.2f]", 
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  }
  else   
  {
    RCLCPP_INFO(get_logger(), "waiting for quatro++ calculation");
    getInitialPose_ = true;
  }

}

std::vector<Eigen::Isometry3d> RelocaliztionNode::generate_initial_guesses(const Eigen::Isometry3d& initial_pose, double trans_noise,double rot_noise)
{
  std::vector<Eigen::Isometry3d> guesses;
    for (int i = 0; i < 10; ++i) {  // generate 10 guess
        Eigen::Vector3d trans = Eigen::Vector3d::Random() * trans_noise;
        Eigen::Vector3d rot_axis = Eigen::Vector3d::Random().normalized();
        Eigen::AngleAxisd rot(rot_noise * Eigen::internal::random(0.0, 1.0), rot_axis);
        
        Eigen::Isometry3d perturbed = initial_pose * Eigen::Translation3d(trans) * rot;
        guesses.emplace_back(perturbed);
    }
    return guesses;
}


void RelocaliztionNode::reset()
{
  pre_result_ =  Eigen::Isometry3d::Identity();
  getInitialPose_ = false;
  doFirstRegistration_ = false;
  // Reset Quatro state so it re-runs after giving a new initial pose
  get_first_tf_from_quatro_ = false;
  using_config_initial_guess_ = false;
  accumulation_counter_ = 0;
  source_cloud_->clear();
  // 重置 transform，防止 timer_pub_tf_callback 发布过时/空的 TF
  transform = geometry_msgs::msg::TransformStamped();
  RCLCPP_WARN(get_logger(), "Reset complete. Quatro will re-run on next initial pose.");
}


void RelocaliztionNode::load_pcd_map(const std::string& map_path){
  if(generate_downsampled_pcd_)
  {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *global_map_) == -1) {
    RCLCPP_ERROR(get_logger(), "Failed to load PCD map: %s", map_path.c_str());
    return;
    }
    RCLCPP_INFO(get_logger(), "Loaded PCD map with %ld points", global_map_->size());
    //downsampling 

    global_map_downsampled_ = voxelgrid_sampling_omp(*global_map_,map_voxel_size_);
    //save downsampled pcd
    if (pcl::io::savePCDFileASCII(downsampled_pcd_file_, *global_map_downsampled_) == -1) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to save downsampled PCD map: %s",
        downsampled_pcd_file_.c_str()
      );
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Successfully saved downsampled map to: %s",
        downsampled_pcd_file_.c_str()
      );
    }
    return;
  }
  // global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(downsampled_pcd_file_, *global_map_downsampled_) == -1) {
    RCLCPP_ERROR(get_logger(), "Failed to load PCD map: %s", downsampled_pcd_file_.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Loaded PCD map with %ld points", global_map_downsampled_->size());
  // //downsampling 

  // global_map_downsampled_ = voxelgrid_sampling_omp(*global_map_,map_voxel_size_);

  auto t_start = std::chrono::steady_clock::now();
  global_map_PointCovariance_ = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*global_map_downsampled_, map_voxel_size_); //do not do downsample
  estimate_covariances_omp(*global_map_PointCovariance_, num_neighbors_, num_threads_);
  //build target kd_tree
  target_tree_ = std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(global_map_PointCovariance_, KdTreeBuilderOMP(num_threads_));
  auto t_end = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
  RCLCPP_INFO(get_logger(), "VoxelGrid downsampling took %ld ms", elapsed_ms);


  RCLCPP_INFO(this->get_logger(),"Downsampled PCD map to %ld points",global_map_PointCovariance_->size());

  sensor_msgs::msg::PointCloud2 output_cloud;
  pcl::toROSMsg(*global_map_downsampled_, output_cloud);
  output_cloud.header.frame_id = "map";
  output_cloud.header.stamp = this->now();
  cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>(output_cloud);
}


void RelocaliztionNode::timer_pub_tf_callback(){
  if (cloud_)
  {   
      // TF only — PCD map is published in the 2s timer_callback to avoid overloading RViz
      if(use_fixed_)
      {
        const bool can_run_with_config_guess =
          use_config_initial_pose_ && prefer_config_initial_pose_ && using_config_initial_guess_;
        if(get_first_tf_from_quatro_ || can_run_with_config_guess)
        {
          if(source_cloud_PointCovariance_)
          {
            if (transform.child_frame_id.empty()) {
              return;  // 不发布空 child_frame_id 的 TF
            }
            transform.header.frame_id = "map";
            transform.header.stamp = this->now();
            tf_broadcaster_->sendTransform(transform);
          }
        }
      }
      else if(!use_quatro_)
      {
        if(source_cloud_PointCovariance_)
        {
          if (transform.child_frame_id.empty()) {
            return;  // 不发布空 child_frame_id 的 TF
          }
          transform.header.frame_id = "map";
          transform.header.stamp = this->now();
          tf_broadcaster_->sendTransform(transform);
        }
      }
        
    }

}


void RelocaliztionNode::timer_callback(){
    if (cloud_)   //test reading pcd
    {   
      cloud_->header.stamp = this->now();
      pointcloud_pub_->publish(*cloud_);  // publish PCD map at 0.5Hz (every 2s), not 100Hz
      if(use_fixed_)  //use tf guess form quatro , small_gicp do global relocalization
      {
        // RCLCPP_INFO(this->get_logger(),"test use fixed method");
        const bool can_run_with_config_guess =
          use_config_initial_pose_ && prefer_config_initial_pose_ && using_config_initial_guess_;
        if(get_first_tf_from_quatro_ || can_run_with_config_guess)
        {
          source_cloud_PointCovariance_ = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*source_cloud_, source_voxel_size_);
          small_gicp::estimate_covariances_omp(*source_cloud_PointCovariance_, num_neighbors_, num_threads_);
          // RCLCPP_INFO(this->get_logger(),"publish pcd map");
          if(source_cloud_PointCovariance_)
          {
            relocalization();
            transform.header.frame_id = "map";  
            transform.header.stamp = this->now();
            tf_broadcaster_->sendTransform(transform);
          }
        }
      }
      else if(!use_quatro_)
      {
        source_cloud_PointCovariance_ = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*source_cloud_, source_voxel_size_);
        small_gicp::estimate_covariances_omp(*source_cloud_PointCovariance_, num_neighbors_, num_threads_);
        // RCLCPP_INFO(this->get_logger(),"publish pcd map");
        if(source_cloud_PointCovariance_)
        {
          relocalization();
          transform.header.frame_id = "map";  
          transform.header.stamp = this->now();
          tf_broadcaster_->sendTransform(transform);
        }
      }
        
    }
}
void RelocaliztionNode::pointcloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 首次收到点云时，如果配置了初始位姿且尚未获取初始位姿，则自动设置
  if (!getInitialPose_ && use_config_initial_pose_) {
    RCLCPP_INFO(get_logger(),
      "Auto-setting initial pose from config: x=%.2f, y=%.2f, z=%.2f, yaw=%.4f",
      initial_pose_x_, initial_pose_y_, initial_pose_z_, initial_pose_yaw_);
    // 从 x, y, z, yaw 构建变换矩阵
    Eigen::Matrix4d initial_transform = Eigen::Matrix4d::Identity();
    double yaw = initial_pose_yaw_;
    initial_transform(0, 0) = cos(yaw);
    initial_transform(0, 1) = -sin(yaw);
    initial_transform(1, 0) = sin(yaw);
    initial_transform(1, 1) = cos(yaw);
    initial_transform(0, 3) = initial_pose_x_;
    initial_transform(1, 3) = initial_pose_y_;
    initial_transform(2, 3) = initial_pose_z_;
    initial_guess_ = Eigen::Isometry3d(initial_transform);
    using_config_initial_guess_ = true;
    getInitialPose_ = true;
    // 不再需要等待 Quatro，直接标记为已获取初始位姿
    // 注意：保留 /initialpose topic 订阅作为备用手动覆盖方式
  }
  if (!getInitialPose_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,  // 1 second
      "Waiting for initial pose..."
    );
    return;
  }
  if(use_fixed_)
  {
    const bool need_quatro_bootstrap =
      !get_first_tf_from_quatro_ &&
      !(use_config_initial_pose_ && prefer_config_initial_pose_ && using_config_initial_guess_);

    // RCLCPP_INFO(this->get_logger(),"test use fixed method");
    if(accumulation_counter_<=2 && need_quatro_bootstrap)
    {
      pcl::fromROSMsg(*msg, *accumulate_cloud_);
      // PCL += 运算符不复制 header，手动保存 frame_id
      if (source_cloud_->header.frame_id.empty()) {
          source_cloud_->header.frame_id = accumulate_cloud_->header.frame_id;
      }
      *source_cloud_+=*accumulate_cloud_;
      accumulation_counter_++;
      if (accumulation_counter_>2)
      {
        source_cloud_downsampled_ = voxelgrid_sampling_omp(*source_cloud_,source_voxel_size_);
        bool if_valid_;
        RCLCPP_INFO(get_logger(), "Starting Quatro align: source=%ld pts, target=%ld pts",
            source_cloud_downsampled_->size(), global_map_downsampled_->size());
        auto t0 = std::chrono::steady_clock::now();
        Eigen::Matrix4d output_tf_ = m_quatro_handler->align(*source_cloud_downsampled_, *global_map_downsampled_,if_valid_);
        auto t1 = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "Quatro align finished in %ld ms, valid=%d",
            std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count(), if_valid_);
        if (if_valid_&&!get_first_tf_from_quatro_)
        {
          RCLCPP_INFO(get_logger(), "Quatro output T translation: [%.4f, %.4f, %.4f]",
              output_tf_(0,3), output_tf_(1,3), output_tf_(2,3));
          publishTransform(output_tf_);
          getInitialPose_ = true;
          initial_guess_ = Eigen::Isometry3d(output_tf_);
          using_config_initial_guess_ = false;
          RCLCPP_INFO(this->get_logger(),"publish tf");
          get_first_tf_from_quatro_ = true;
        }
        else {
          RCLCPP_WARN(get_logger(), "Quatro failed, retrying...");
          accumulation_counter_ = 0; //reset, accumulate again
        }
      }
      return;
    }
    pcl::fromROSMsg(*msg, *source_cloud_);

    source_cloud_downsampled_ = voxelgrid_sampling_omp(*source_cloud_,source_voxel_size_);

    // bool if_valid_;
    // Eigen::Matrix4d output_tf_ = m_quatro_handler->align(*source_cloud_downsampled_, *global_map_downsampled_,if_valid_);
    // if (if_valid_&&!get_first_tf_from_quatro_)
    // {
    //   publishTransform(output_tf_);
    //   getInitialPose_ = true;
    //   initial_guess_ = Eigen::Isometry3d(output_tf_);
    //   RCLCPP_INFO(this->get_logger(),"publish tf");
    //   get_first_tf_from_quatro_ = true;
    // }

  }
  else if(!use_quatro_)
  {
    pcl::fromROSMsg(*msg, *source_cloud_);    
    // source_tree_ = std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(source_cloud_PointCovariance_, KdTreeBuilderOMP(num_threads_));
    // relocalization(source_cloud_);
    // relocalization();
  }
  else
  {
    
    pcl::fromROSMsg(*msg, *source_cloud_);
    source_cloud_downsampled_ = voxelgrid_sampling_omp(*source_cloud_,source_voxel_size_);
    bool if_valid_;
    Eigen::Matrix4d output_tf_ = m_quatro_handler->align(*source_cloud_downsampled_, *global_map_downsampled_,if_valid_);
    if (if_valid_)
    {
      publishTransform(output_tf_);
      RCLCPP_INFO(this->get_logger(),"publish tf");
    }
    else  RCLCPP_INFO(this->get_logger(),"invalid");

  }
}

void RelocaliztionNode::publishTransform(const Eigen::Matrix4d& transform_matrix) {
  transform.header.stamp = this->now();
  transform.header.frame_id = "map";   
  transform.child_frame_id = source_cloud_->header.frame_id;    
  transform.transform.translation.x = transform_matrix(0, 3);
  transform.transform.translation.y = transform_matrix(1, 3);
  transform.transform.translation.z = transform_matrix(2, 3);

  Eigen::Matrix3d rotation = transform_matrix.block<3,3>(0, 0);
  Eigen::Quaterniond quat(rotation);
  transform.transform.rotation.x = quat.x();
  transform.transform.rotation.y = quat.y();
  transform.transform.rotation.z = quat.z();
  transform.transform.rotation.w = quat.w();

  tf_broadcaster_->sendTransform(transform);
}

int fail_counter = 0;
/// @brief Example to directly feed pcl::PointCloud<pcl::PointCovariance> to small_gicp::Registration.
void RelocaliztionNode::relocalization() {
  if (!getInitialPose_) {
    RCLCPP_WARN(get_logger(), "No initial pose received. Skipping relocalization.");
    return;
  }
  
  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>.
  // pcl::PointCloud<pcl::PointCovariance>::Ptr target = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*raw_target, 0.25);
  // pcl::PointCloud<pcl::PointCovariance>::Ptr source = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*raw_source, 0.25);

  // Estimate covariances of points.
  // const int num_threads = num_threads_;
  // const int num_neighbors = num_neighbors_;
  // estimate_covariances_omp(*global_map_PointCovariance_, num_neighbors, num_threads);
  // estimate_covariances_omp(*source_cloud_PointCovariance_, num_neighbors, num_threads);

  // Create KdTree for target and source.
  // auto target_tree = std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(global_map_PointCovariance_, KdTreeBuilderOMP(num_threads));
  // auto source_tree = std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(source, KdTreeBuilderOMP(num_threads));

  Registration<GICPFactor, ParallelReductionOMP> registration;
  registration.reduction.num_threads = num_threads_;
  registration.rejector.max_dist_sq = max_dist_sq_;
  
  if(doFirstRegistration_){
      RCLCPP_INFO(get_logger(), "ICP input: source=%ld pts, target=%ld pts",
          source_cloud_PointCovariance_->size(), global_map_PointCovariance_->size());
      RCLCPP_INFO(get_logger(), "ICP initial_guess (pre_result_) translation: [%.4f, %.4f, %.4f]",
          pre_result_.translation().x(), pre_result_.translation().y(), pre_result_.translation().z());
      result = registration.align(*global_map_PointCovariance_, *source_cloud_PointCovariance_, *target_tree_, pre_result_);
      RCLCPP_INFO(get_logger(), "ICP result: converged=%d, error=%.4f, T_translation=[%.4f, %.4f, %.4f]",
          result.converged, result.error,
          result.T_target_source.translation().x(),
          result.T_target_source.translation().y(),
          result.T_target_source.translation().z());
  }
  // else  
  // { 
  //   std::vector<RegistrationResult> results;
  //   #pragma omp parallel for num_threads(4)
  //   for(const auto& guess_:guesses_)
  //   {
  //     result = registration.align(*global_map_PointCovariance_, *source_cloud_PointCovariance_, *target_tree_, guess_);
  //     RegistrationResult rs_;
  //     rs_.converged = result.converged;
  //     rs_.error = result.error;
  //     rs_.transformation = result.T_target_source;
  //     #pragma omp critical
  //     results.emplace_back(rs_);
  //   }
  //   auto best_result = std::min_element(results.begin(), results.end(), 
  //   [](const auto& a, const auto& b) { return a.error < b.error; });
  //   pre_result_=best_result->transformation;
  //   // if(best_result->error>20.0&&!doFirstRegistration_)
  //   // {
  //   //   RCLCPP_INFO(get_logger(), "cannot do first registration,reset,result error:%f",best_result->error);
  //   //   reset();
  //   //   return;
  //   // }
  //   // else pre_result_=best_result->transformation;
  // }
  else
  {
    RCLCPP_INFO(get_logger(), "ICP input: source=%ld pts, target=%ld pts",
        source_cloud_PointCovariance_->size(), global_map_PointCovariance_->size());
    RCLCPP_INFO(get_logger(), "ICP initial_guess translation: [%.4f, %.4f, %.4f]",
        initial_guess_.translation().x(), initial_guess_.translation().y(), initial_guess_.translation().z());
    result = registration.align(*global_map_PointCovariance_, *source_cloud_PointCovariance_, *target_tree_, initial_guess_);
    RCLCPP_INFO(get_logger(), "ICP result: converged=%d, error=%.4f, T_translation=[%.4f, %.4f, %.4f]",
        result.converged, result.error,
        result.T_target_source.translation().x(),
        result.T_target_source.translation().y(),
        result.T_target_source.translation().z());
    if (result.converged && using_config_initial_guess_) {
      const Eigen::Vector3d jump =
        result.T_target_source.translation() - initial_guess_.translation();
      const double jump_norm = jump.norm();
      RCLCPP_INFO(get_logger(),
        "First ICP translation jump from config guess: %.4f m (limit=%.2f m)",
        jump_norm, first_registration_max_translation_jump_);
      if (jump_norm > first_registration_max_translation_jump_) {
        RCLCPP_WARN(get_logger(),
          "Rejecting first ICP result: jump %.4f m exceeds limit %.2f m. "
          "Likely matched to symmetric/opposite-side region.",
          jump_norm, first_registration_max_translation_jump_);
        result.converged = false;
      }
    }
    if(!result.converged&&!doFirstRegistration_)
    {
      RCLCPP_INFO(get_logger(), "cannot do first registration,reset,result error:%f",result.error);
      reset();
      transform.header.frame_id = "map";
      transform.child_frame_id = source_cloud_->header.frame_id;
      transform.transform = tf2::eigenToTransform(initial_guess_).transform;
      return;
    }
  }
  
  // publish map->odom tf
  if (result.converged) {
    // pre_result_ = result.T_target_source.inverse();
    pre_result_ = result.T_target_source;
    doFirstRegistration_ = true;
    Eigen::Isometry3d T_map_odom = pre_result_;
    
    // publish transform
    transform.header.stamp = this->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = source_cloud_->header.frame_id;
    transform.transform = tf2::eigenToTransform(T_map_odom).transform;
        RCLCPP_INFO(get_logger(), "Published map->odom transform,result error:%f",result.error);
  } else {
    fail_counter++;
    if(fail_counter>1)
    {
      fail_counter=0;
      reset();
      RCLCPP_ERROR(get_logger(), "fail upto the threshold,reset,please set initalpose again");
    }
    
    RCLCPP_ERROR(get_logger(), "Relocalization failed to converge,result error:%f",result.error);
  }

  // 第一次结果的逆矩阵（map->odom）
  // std::cout << "--- T_map_odom ---" << std::endl << result.T_target_source.inverse().matrix() << std::endl;

  // std::cout << "--- T_target_source ---" << std::endl << result.T_target_source.matrix() << std::endl;
  // std::cout << "converged:" << result.converged << std::endl;
  // std::cout << "error:" << result.error << std::endl;
  // std::cout << "iterations:" << result.iterations << std::endl;
  // std::cout << "num_inliers:" << result.num_inliers << std::endl;
  // std::cout << "--- H ---" << std::endl << result.H << std::endl;
  // std::cout << "--- b ---" << std::endl << result.b.transpose() << std::endl;

  // // Because this usage exposes all preprocessed data, you can easily re-use them to obtain the best efficiency.
  // auto result2 = registration.align(*source, *target, *source_tree, Eigen::Isometry3d::Identity());

  // std::cout << "--- T_target_source ---" << std::endl << result2.T_target_source.inverse().matrix() << std::endl;
}


}
