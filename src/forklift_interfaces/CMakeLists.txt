cmake_minimum_required(VERSION 3.8)
project(forklift_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StringWithJson.srv"
)

ament_export_dependencies(rosidl_default_runtime)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
