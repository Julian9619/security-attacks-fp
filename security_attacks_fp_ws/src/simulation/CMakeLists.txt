cmake_minimum_required(VERSION 3.0.2)
project(collision-simulation)

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

add_library(libcarPlugin src/carPlugin.cpp)
target_link_libraries(libcarPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(libpostbox1Plugin src/postbox1Plugin.cpp)
target_link_libraries(libpostbox1Plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(libpostbox2Plugin src/postbox2Plugin.cpp)
target_link_libraries(libpostbox2Plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

