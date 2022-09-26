cmake_minimum_required(VERSION 3.1)
project(read_calibrated)

set(CMAKE_CXX_STANDARD 17)

set(PROJECT_CATKIN_DEPS
  roscpp
  nav_msgs
  pcl_conversions
  pcl_ros
  )

find_package(catkin REQUIRED
  COMPONENTS
  ${PROJECT_CATKIN_DEPS}
  )

find_package(PCL REQUIRED)
find_package(cartographer REQUIRED)
find_package(Protobuf)
find_package(Boost COMPONENTS iostreams REQUIRED)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  ${PROJECT_CATKIN_DEPS}
  )

add_definitions(${catkin_DEFINITIONS})

set(cartographer_INCLUDE_INTERNAL_DIRS "${PROJECT_SOURCE_DIR}/../cartographer")
include_directories(
  include
  ${cartographer_INCLUDE_INTERNAL_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/read_calibrated.cpp)
add_executable(${PROJECT_NAME}_filter src/filter_node.cc)
add_executable(${PROJECT_NAME}_initial src/check_initial_pose.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  cartographer
  ${PROTOBUF_LIBRARY}
  ${BOOST_LIBRARIES}
  )

target_link_libraries(${PROJECT_NAME}_filter
  ${catkin_LIBRARIES}
  cartographer
  ${PROTOBUF_LIBRARY}
  )

target_link_libraries(${PROJECT_NAME}_initial
  ${catkin_LIBRARIES}
  cartographer
  ${PROTOBUF_LIBRARY}
  )
