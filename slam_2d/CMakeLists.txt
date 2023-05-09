cmake_minimum_required(VERSION 3.5)
project(slam_2d)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs)

set(dependencies
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  Eigen3)

add_library(fastslam SHARED
  src/fastslam.cpp
  src/particle.cpp
  src/occupancy_grid.cpp
  src/slam_utils.cpp)

target_include_directories(fastslam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fastslam
  ${dependencies})

add_executable(slam_node
  src/slam_node.cpp)
target_include_directories(slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(slam_node
  fastslam)
ament_target_dependencies(slam_node
  ${dependencies})

install(TARGETS fastslam
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS slam_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_slam_test
    test/slam_test.cpp)
  target_include_directories(${PROJECT_NAME}_slam_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_slam_test
    ${dependencies})
  target_link_libraries(${PROJECT_NAME}_slam_test
    fastslam)
endif()

ament_package()
