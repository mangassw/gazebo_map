cmake_minimum_required(VERSION 3.0.2)
project(rpp_pkg)

find_package(catkin REQUIRED COMPONENTS
  angles
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /opt/ros/noetic/include  # 强制添加ROS头文件根目录
)

add_library(${PROJECT_NAME}
  src/controller.cpp
  src/rpp_controller.cpp
  src/rpp_math.cpp  
  src/rpp_vec2d.cpp
  
)

# 链接依赖
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_compile_options(-finput-charset=UTF-8 -fexec-charset=UTF-8)




