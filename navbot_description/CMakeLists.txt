cmake_minimum_required(VERSION 2.8.3)
project(navbot_description)

# Compile as C++11, for ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  gazebo_ros
  dynamic_reconfigure
)

find_package(gazebo REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/GazeboTrafficLight.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_traffic_light
  CATKIN_DEPENDS rospy roscpp gazebo_ros dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gazebo_traffic_light src/GazeboTrafficLight.cpp)
add_dependencies(gazebo_traffic_light ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_traffic_light ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Set Gazebo CXX Flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Install plugin library
install(TARGETS gazebo_traffic_light
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install other necessary files like launch files, yaml configurations, etc.
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY cfg/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

