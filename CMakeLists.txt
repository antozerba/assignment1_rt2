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
find_package(rosidl_default_runtime REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interface REQUIRED)

#executables
add_executable(target_interface "src/ui_node.cpp")

##COMPONENTS
add_library(controller_component SHARED src/controller.cpp)
rclcpp_components_register_node(controller_component
  PLUGIN "target_controller::TargetController"
  EXECUTABLE controller)

# dependencies for executables / componentes
ament_target_dependencies(target_interface rclcpp rclcpp_action tf2 std_msgs tf2_ros geometry_msgs custom_interface)
ament_target_dependencies(controller_component rclcpp rclcpp_action tf2 tf2_geometry_msgs tf2_ros custom_interface geometry_msgs rclcpp_components nav_msgs)


# launch file
install( DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


#install
install(TARGETS
  target_interface
  controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_package()

