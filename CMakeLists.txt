cmake_minimum_required(VERSION 2.8.3)
project(read_calibrated)

find_package(catkin REQUIRED)
find_package(cartographer REQUIRED)
find_package(Protobuf 3.4.1 EXACT REQUIRED CONFIG HINTS /opt/enway/lib/protobuf-3.4.1/lib/x86_64-linux-gnu/cmake/protobuf)
set(PROTOBUF_INCLUDE_DIRS "/opt/enway/lib/protobuf-3.4.1/include")
set(PROTOBUF_LIBRARY "/opt/enway/lib/protobuf-3.4.1/lib/x86_64-linux-gnu/libprotobuf.a")

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
  )

target_link_libraries(${PROJECT_NAME}_filter
  ${catkin_LIBRARIES}
  cartographer
  ${PROTOBUF_LIBRARY}
  )
