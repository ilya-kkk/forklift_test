cmake_minimum_required(VERSION 3.8)
project(vda5050_3_driver)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/vda5050_3_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})

ament_package()
