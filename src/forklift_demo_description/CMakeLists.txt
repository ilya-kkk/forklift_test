cmake_minimum_required(VERSION 3.8)
project(forklift_demo_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    behavior_trees
    config
    launch
    models
    rviz
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
