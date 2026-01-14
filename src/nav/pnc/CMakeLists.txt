cmake_minimum_required(VERSION 3.0.2)
project(xj_robot_pnc)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        costmap_2d
        geometry_msgs
        message_generation
        mbf_msgs
        nav_msgs
        roscpp
        roslaunch
        sensor_msgs
        tf2
        tf2_ros
)

add_service_files(
        DIRECTORY srv
        FILES
        exe_path.srv
        record_start.srv
        record_stop.srv
        xj_robot_task.srv
        nav_goal.srv
)

generate_messages()

catkin_package(CATKIN_DEPENDS
        actionlib_msgs
        costmap_2d
        geometry_msgs
        message_runtime
        mbf_msgs
        nav_msgs
        roscpp
        sensor_msgs
        tf2
        tf2_ros
        )
include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)
add_executable(xj_robot_mbf_bridge
        node/mbf_bridge_node.cpp)
add_dependencies(xj_robot_mbf_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xj_robot_mbf_bridge ${catkin_LIBRARIES})

# add_executable(xj_robot_state_machine
#         node/state_machine_node.cpp
#         src/state_machine.cpp)
# add_dependencies(xj_robot_state_machine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(xj_robot_state_machine ${catkin_LIBRARIES})

# add_executable(xj_robot_run_tasks
#         node/run_tasks_node.cpp
#         src/run_tasks.cpp)
# add_dependencies(xj_robot_run_tasks ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(xj_robot_run_tasks ${catkin_LIBRARIES})
# add_executable(run_waypoint
#         node/run_waypoint_node.cpp
#         src/run_waypoint.cpp)
# add_dependencies(run_waypoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(run_waypoint ${catkin_LIBRARIES})
add_executable(nav_to_goal
        node/nav_to_goal_node.cpp
        src/nav_to_goal.cpp)
add_dependencies(nav_to_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nav_to_goal ${catkin_LIBRARIES})