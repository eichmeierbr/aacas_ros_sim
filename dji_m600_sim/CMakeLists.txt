cmake_minimum_required(VERSION 2.8.3)
project(dji_m600_sim)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  tf
  roscpp
  robot_state_publisher
  gazebo_ros
  rviz_visual_tools
  message_generation
)


add_service_files(
  DIRECTORY srv
  FILES
  DroneTaskControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(gazebo REQUIRED)
catkin_package(
)

include_directories(
	${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
	${GAZEBO_LIRARY_DIRS} ${catkin_LIBRARY_DIRS}
)



add_executable(rviz_collision_publisher src/rviz_collision_publisher.cpp)
add_executable(rviz_collision_publisher_no_gazebo src/rviz_collision_publisher_no_gazebo.cpp)
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(rviz_collision_publisher ${catkin_LIBRARIES})
target_link_libraries(rviz_collision_publisher_no_gazebo ${catkin_LIBRARIES})
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

install(TARGETS rviz_collision_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
install(DIRECTORY rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)