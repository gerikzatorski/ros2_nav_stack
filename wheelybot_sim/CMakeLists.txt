cmake_minimum_required(VERSION 3.5)
project(wheelybot_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    config
    worlds
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
