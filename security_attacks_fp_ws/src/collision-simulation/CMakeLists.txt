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

catkin_package(
  ##INCLUDE_DIRS include
  ##CATKIN_DEPENDS gazebo_ros roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(myCollisionModelPlugin src/myCollisionModelPlugin.cpp)
target_link_libraries(myCollisionModelPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_library(myCollisionSensorPlugin src/myCollisionSensorPlugin.cpp)
target_link_libraries(myCollisionSensorPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(myRecieverPlugin src/myRecieverPlugin.cpp)
target_link_libraries(myRecieverPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(myMotorPlugin src/myMotorPlugin.cpp)
target_link_libraries(myMotorPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(postbox1Plugin src/postbox1Plugin.cpp)
target_link_libraries(postbox1Plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(postbox2Plugin src/postbox2Plugin.cpp)
target_link_libraries(postbox2Plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_executable(attacker src/AttackerApp.cpp)
target_link_libraries(attacker ${catkin_LIBRARIES})
##add_dependencies(talker beginner_tutorials_generate_messages_cpp)