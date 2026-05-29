cmake_minimum_required(VERSION 3.8)
project(cmd_vel_arcestrator)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS scripts/cmd_vel_arcestrator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_arcestrator
)

ament_package()
