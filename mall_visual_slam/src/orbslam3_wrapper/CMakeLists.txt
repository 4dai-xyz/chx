cmake_minimum_required(VERSION 3.8)
project(orbslam3_wrapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找ROS依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# --- ORB-SLAM3 路径 ---
get_filename_component(ROS2_ORBSLAM3_REPO "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(DEFAULT_ORB_SLAM3_DIR "${ROS2_ORBSLAM3_REPO}/Opensource code/ORB_SLAM3-master")
set(LEGACY_ORB_SLAM3_DIR "/home/ros/ORB-SLAM3-STEREO-FIXED")

if(DEFINED ENV{ORB_SLAM3_DIR})
  set(ORB_SLAM3_DIR "$ENV{ORB_SLAM3_DIR}" CACHE PATH "Path to the ORB-SLAM3 source/build tree" FORCE)
elseif(NOT DEFINED ORB_SLAM3_DIR OR ORB_SLAM3_DIR STREQUAL LEGACY_ORB_SLAM3_DIR)
  set(ORB_SLAM3_DIR "${DEFAULT_ORB_SLAM3_DIR}" CACHE PATH "Path to the ORB-SLAM3 source/build tree" FORCE)
else()
  set(ORB_SLAM3_DIR "${ORB_SLAM3_DIR}" CACHE PATH "Path to the ORB-SLAM3 source/build tree")
endif()

if(DEFINED ENV{ORB_SLAM3_LIBRARY})
  set(ORB_SLAM3_LIBRARY "$ENV{ORB_SLAM3_LIBRARY}" CACHE FILEPATH "Path to libORB_SLAM3.so" FORCE)
else()
  set(ORB_SLAM3_LIBRARY "${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so" CACHE FILEPATH "Path to libORB_SLAM3.so" FORCE)
endif()

message(STATUS "Using ORB_SLAM3_DIR=${ORB_SLAM3_DIR}")
message(STATUS "Using ORB_SLAM3_LIBRARY=${ORB_SLAM3_LIBRARY}")

if(NOT EXISTS "${ORB_SLAM3_DIR}/include/System.h")
  message(FATAL_ERROR
    "ORB-SLAM3 headers were not found at ${ORB_SLAM3_DIR}/include. "
    "Set ORB_SLAM3_DIR to the migrated ORB-SLAM3 tree.")
endif()

if(NOT EXISTS "${ORB_SLAM3_LIBRARY}")
  message(FATAL_ERROR
    "libORB_SLAM3.so was not found at ${ORB_SLAM3_LIBRARY}. "
    "Build ORB-SLAM3 first or set ORB_SLAM3_LIBRARY.")
endif()

# --- 查找 Pangolin 库 ---
find_package(Pangolin REQUIRED)

# 2. 创建可执行文件
add_executable(mono_node src/mono_node.cpp)

# 3. 【关键】使用 target_include_directories 显式指定路径
# 这个方法专门针对 mono_node 这个可执行文件，不会被全局设置覆盖
target_include_directories(mono_node PRIVATE
  ${ORB_SLAM3_DIR}             # 根目录
  ${ORB_SLAM3_DIR}/include     # System.h 所在的目录
  ${ORB_SLAM3_DIR}/include/CameraModels     # 包含 GeometricCamera.h 的目录
  ${EIGEN3_INCLUDE_DIRS}       # Eigen3 目录
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus # Sophus 目录
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2 # DBoW2 目录
  ${ORB_SLAM3_DIR}/Thirdparty/g2o   # g2o 目录
)

# 4. 自动处理 ROS 库链接
ament_target_dependencies(mono_node
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  nav_msgs
  std_msgs
)

# 5. 链接 ORB-SLAM3 库
target_link_libraries(mono_node
  ${ORB_SLAM3_LIBRARY}
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  pthread
)

# 6. 安装
install(TARGETS
  mono_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
