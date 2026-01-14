cmake_minimum_required(VERSION 3.0.2)
project(xj_robot_simu)


find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	message_runtime
)

add_message_files(FILES
		carpose.msg
		fusion_analysis.msg
		)
generate_messages()

catkin_package(CATKIN_DEPENDS message_generation message_runtime roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(msg_bridge_node node/msg_bridge_node.cpp src/msg_bridge.cpp)
add_dependencies(msg_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(msg_bridge_node ${catkin_LIBRARIES})


add_executable(new_msg_bridge src/new_msg_bridge.cpp)
add_dependencies(new_msg_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(new_msg_bridge ${catkin_LIBRARIES})

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)
install(PROGRAMS
		teleop/teleop_key
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)
install(DIRECTORY include
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.h"
		)
install(DIRECTORY launch rviz urdf worlds
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
		)