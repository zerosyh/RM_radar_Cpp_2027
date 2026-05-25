cmake_minimum_required(VERSION 3.10)
project(registration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  rclcpp 
  sensor_msgs 
  tf2 
  tf2_ros 
  tf2_eigen
  pcl_conversions
  geometry_msgs
  pcl_ros
  quatro
)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)

if(OpenMP_CXX_FOUND)
  add_definitions(-DOPENMP_AVAILABLE)  # 定义宏告诉small_gicp已启用OpenMP
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


ament_auto_add_library(registration
        src/registration_node.cpp
)



ament_auto_add_executable(registration_node src/main.cpp src/registration_node.cpp)
# link library

target_include_directories(registration_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(registration_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  rclcpp::rclcpp
  OpenMP::OpenMP_CXX
  teaserpp::teaser_registration 
  teaserpp::teaser_io
)

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        params
)
