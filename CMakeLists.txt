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

#action generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Target.action"
)
ament_export_dependencies(rosidl_default_runtime)

#executables
add_executable(target_interface "src/ui_node.cpp")
# add_executable(controller "src/controller.cpp")

ament_target_dependencies(target_interface rclcpp rclcpp_action tf2 tf2_ros geometry_msgs)
# ament_target_dependencies(controller rclcpp rclcpp_action tf2 tf2_ros geometry_msgs rclcpp_components)

##COMPONENTS
add_library(controller_component SHARED src/controller.cpp)
rclcpp_components_register_node(controller_component
  PLUGIN "assignment1_rt2::Controller"
  EXECUTABLE controller)

rosidl_target_interfaces(
  target_interface 
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)




#install
install(TARGETS
  target_interface
  controller_component
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

