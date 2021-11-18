cmake_minimum_required(VERSION 3.0.2)
project(lidar-with-velocity)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_COMPILER /usr/bin/g++)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${YAML_CPP_INCLUDE_DIR})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_HOME_DIRECTORY}/include/)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rosmsg
  rospy
  sensor_msgs
  pcl_ros
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs  
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar-with-velocity
  CATKIN_DEPENDS roscpp rosmsg rospy
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(main_ros 
  src/assignment/main.cpp
  src/assignment/assignment.cpp 
  src/assignment/frame.cpp 
  src/assignment/config.cpp
  src/assignment/tracker.cpp 
  src/assignment/Hungarian.cpp
)
target_link_libraries(main_ros 
  ${catkin_LIBRARIES} 
  ${PYTHON_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBS} 
  yaml-cpp
)
