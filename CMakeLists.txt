cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

find_package(catkin 
	REQUIRED COMPONENTS
	 roscpp 
	 rospy 
	 std_msgs 
	 nav_msgs 
 	 message_generation)

add_message_files(FILES CheckpointDistance.msg)

# add_service_files(FILES)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_planner
  CATKIN_DEPENDS 
   message_runtime
   roscpp
   rospy
   std_msgs
   nav_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(path_planner_node src/path_planner.cpp)
target_link_libraries(path_planner_node ${catkin_LIBRARIES})
add_dependencies(path_planner_node path_planner_generate_messages_cpp)


