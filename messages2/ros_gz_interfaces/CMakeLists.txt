cmake_minimum_required(VERSION 3.8)
project(ros_gz_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ParamVec.msg"
  DEPENDENCIES std_msgs rcl_interfaces
 )

install(
  FILES bridge.yaml
  DESTINATION share/${PROJECT_NAME})


ament_export_dependencies(rosidl_default_runtime)

ament_package()
