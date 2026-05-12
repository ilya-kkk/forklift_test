cmake_minimum_required(VERSION 3.8)
project(palette_picking)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
