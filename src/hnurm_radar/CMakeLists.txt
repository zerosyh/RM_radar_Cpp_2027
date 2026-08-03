cmake_minimum_required(VERSION 3.18)  # 需要 3.18 以支持 CUDA 作为语言
project(hnurm_radar LANGUAGES CXX CUDA)  # 启用 CUDA 语言

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------- 依赖查找 ---------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.hpp
    DOWNLOAD_NO_EXTRACT TRUE
  )
  FetchContent_MakeAvailable(nlohmann_json)
  include_directories(${nlohmann_json_SOURCE_DIR})
endif()

find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(detect_result REQUIRED)

# ONNX Runtime 查找（保留以备将来使用）
find_package(onnxruntime QUIET)

# =============== 海康相机 MVS SDK ===============
set(MVS_ROOT "/opt/MVS")
find_library(MVS_LIBRARY
  NAMES MvCameraControl
  PATHS ${MVS_ROOT}/lib/64
  NO_DEFAULT_PATH
)
if(NOT MVS_LIBRARY)
  message(FATAL_ERROR "未找到 libMvCameraControl.so，路径：${MVS_ROOT}/lib/64")
endif()
include_directories(${MVS_ROOT}/include)

# =============== CUDA 与 TensorRT ===============
# 使用 CUDAToolkit
find_package(CUDAToolkit REQUIRED)

# TensorRT 路径：优先使用环境变量 TENSORRT_DIR，否则使用默认路径
if(DEFINED ENV{TENSORRT_DIR})
    set(TENSORRT_DIR $ENV{TENSORRT_DIR})
else()
    set(TENSORRT_DIR "/home/syh/TensorRT-10.13.2.6" CACHE PATH "TensorRT 安装目录")
endif()
set(TENSORRT_INCLUDE_DIR ${TENSORRT_DIR}/include)
set(TENSORRT_LIB_DIR ${TENSORRT_DIR}/lib)

# 查找 nvinfer 库
find_library(TENSORRT_LIB nvinfer PATHS ${TENSORRT_LIB_DIR} NO_DEFAULT_PATH REQUIRED)
find_library(TENSORRT_PLUGIN_LIB nvinfer_plugin PATHS ${TENSORRT_LIB_DIR} NO_DEFAULT_PATH REQUIRED)

# --------------- 可执行文件 ---------------

# --- hnurm_radr_node ---


# --- detect 节点（已拆分：detect.cpp + infer.cu + HomographyTransformer.cpp）---
add_executable(detect
  src/detect.cpp
  src/infer.cu
  src/HomographyTransformer.cpp
  src/tracker.cpp
)

target_include_directories(detect PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TENSORRT_INCLUDE_DIR}
)
target_compile_features(detect PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(detect PRIVATE)

# 设置 CUDA 架构（针对 RTX 5060 计算能力 12.0）
set_target_properties(detect PROPERTIES CUDA_ARCHITECTURES "120")

ament_target_dependencies(
  detect
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "cv_bridge"
  "detect_result"
  "ament_index_cpp"
)

target_link_libraries(detect
  ${OpenCV_LIBS}
  yaml-cpp
  nlohmann_json::nlohmann_json
  CUDA::cudart
  ${TENSORRT_LIB}
  ${TENSORRT_PLUGIN_LIB}
)

# --- lidar ---
add_executable(lidar src/lidar.cpp)
target_include_directories(lidar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lidar PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  lidar
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
  "tf2_ros"
  "tf2_eigen"
)
target_link_libraries(lidar ${PCL_LIBRARIES} yaml-cpp Eigen3::Eigen)

# --- radar 融合定位节点 ---
add_executable(radar
  src/radar.cpp
)

target_include_directories(radar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(radar PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  radar
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
  "tf2_ros"
  "tf2_eigen"
  "detect_result"
  "yaml-cpp"
)
target_link_libraries(radar ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp Eigen3::Eigen)

# --- hik_camera_node ---
add_executable(hik_camera_node src/hik_camera_node.cpp)
target_include_directories(hik_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hik_camera_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  hik_camera_node
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
)
target_link_libraries(hik_camera_node ${MVS_LIBRARY} ${OpenCV_LIBS})
# --- display_panel 小地图可视化 ---
add_executable(display_panel src/display_panel.cpp)

target_include_directories(display_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(display_panel PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  display_panel
  "rclcpp"
  "sensor_msgs"
  "detect_result"
  "cv_bridge"
)
target_link_libraries(display_panel ${OpenCV_LIBS} yaml-cpp)

# --- sentry_decision 哨兵决策辅助 ---
add_executable(sentry_decision src/sentry_decision.cpp)
target_include_directories(sentry_decision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sentry_decision PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  sentry_decision
  "rclcpp"
  "detect_result"
)
target_link_libraries(sentry_decision yaml-cpp ${OpenCV_LIBS})


# --- judge_messager 裁判系统通信 ---
add_executable(judge_messager src/judge_messager.cpp)
target_include_directories(judge_messager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(judge_messager PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  judge_messager
  "rclcpp"
  "detect_result"
)
target_link_libraries(judge_messager yaml-cpp)

# --------------- 安装 ---------------
install(TARGETS  lidar hik_camera_node detect radar display_panel sentry_decision judge_messager
  DESTINATION lib/${PROJECT_NAME})

# 安装模型和配置等资源
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/model)
  install(DIRECTORY model
    DESTINATION share/${PROJECT_NAME})
elseif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../model)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../model
    DESTINATION share/${PROJECT_NAME}/model)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/config)
elseif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../configs)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../configs
    DESTINATION share/${PROJECT_NAME}/config)
elseif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../config)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../config
    DESTINATION share/${PROJECT_NAME}/config)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()