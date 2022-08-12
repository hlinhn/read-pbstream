cmake_minimum_required(VERSION 3.1)
project(read_calibrated)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED)
find_package(cartographer REQUIRED)
find_package(Protobuf)
find_package(Boost COMPONENTS iostreams REQUIRED)

catkin_package()

set(cartographer_INCLUDE_INTERNAL_DIRS "/home/linh/repos/utils/src/cartographer")
include_directories(
  include
  ${cartographer_INCLUDE_INTERNAL_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/read_calibrated.cpp)
add_executable(${PROJECT_NAME}_filter src/filter_node.cc)

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
