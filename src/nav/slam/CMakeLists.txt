cmake_minimum_required(VERSION 3.0.2)
project(xj_robot_slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
  std_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf_conversions
  livox_ros_driver2
  hdl_localization
  geometry_msgs
  geographic_msgs
)
find_package(GeographicLib REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xj_robot_slam
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
# add_message_files(
#   FILES
#   PositionAccuracy.msg
# )
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)
add_executable(calc_odom src/calc_odom.cpp)
add_dependencies(calc_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calc_odom ${catkin_LIBRARIES})
add_executable(pcd2livox src/pcd2livox.cpp)
add_dependencies(pcd2livox ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd2livox ${catkin_LIBRARIES})
add_executable(base_link_pub src/base_link_pub.cpp)
target_link_libraries(base_link_pub
  ${catkin_LIBRARIES}
)
add_executable(rtk2enu src/rtk2enu.cpp)

# 链接库
target_link_libraries(rtk2enu
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

add_dependencies(rtk2enu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(base_link_change src/base_link_change.cpp)

# 链接库
target_link_libraries(base_link_change
  ${catkin_LIBRARIES}
)

add_dependencies(base_link_change ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})