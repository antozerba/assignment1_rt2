cmake_minimum_required(VERSION 3.8)
project(assignment1_rt2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

add_executable(target_interface "src/ui_node.cpp")
ament_target_dependencies(target_interface rclcpp rclcpp_action tf2 tf2_ros geometry_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Target.action"
)
install(TARGETS
  target_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

