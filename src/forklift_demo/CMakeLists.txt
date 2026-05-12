cmake_minimum_required(VERSION 3.8)
project(forklift_demo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    models
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
