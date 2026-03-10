cmake_minimum_required(VERSION 3.8)
project(obstacle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation kdtree)

add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)

target_include_directories(obstacle_detector_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(obstacle_detector_node ${PCL_LIBRARIES})

ament_target_dependencies(obstacle_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

install(TARGETS obstacle_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
