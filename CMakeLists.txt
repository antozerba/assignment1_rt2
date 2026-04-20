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
add_executable(controller_node "src/controller.cpp")

##COMPONENTS
add_library(controller_component SHARED src/controller.cpp)
rclcpp_components_register_node(controller_component
  PLUGIN "target_controller::TargetController"
  EXECUTABLE controller)

# dependencies for executables / componentes
ament_target_dependencies(target_interface rclcpp rclcpp_action tf2 tf2_ros geometry_msgs)
ament_target_dependencies(controller_component rclcpp rclcpp_action tf2 tf2_ros geometry_msgs rclcpp_components)
ament_target_dependencies(controller_node rclcpp rclcpp_action tf2 tf2_ros geometry_msgs rclcpp_components)


## interfaces DEPRECATED
# rosidl_target_interfaces(
#   target_interface 
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )
# rosidl_target_interfaces(
#   controller_component 
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

#interfaces 
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(target_interface "${cpp_typesupport_target}")
target_link_libraries(controller_component "${cpp_typesupport_target}")
target_link_libraries(controller_node "${cpp_typesupport_target}")

# launch file
install( DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


#install
install(TARGETS
  target_interface
  controller_component
  controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_package()

