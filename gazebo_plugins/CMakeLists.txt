cmake_minimum_required(VERSION 3.0.2)
project(gazebo_ros_1)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  genmsg
  rospy
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

##generate_messages(DEPENDENCIES std_msgs)

##catkin_package(
  ##INCLUDE_DIRS include
  ##CATKIN_DEPENDS gazebo_ros roscpp
##)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(your_model-plugin_name src/your_model-plugin_src_name.cpp)
target_link_libraries(your_model-plugin_name ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_library(your_sensor-plugin_name src/your_sensor-plugin_src_name.cpp)
target_link_libraries(your_sensor-plugin_name ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

##wiederhole für jedes plugin das du hinzufügen willst


