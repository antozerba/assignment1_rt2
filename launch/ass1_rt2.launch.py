from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    bme_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bme_gazebo_sensors'),
                'launch',
                'spawn_robot_ex.launch.py'
            )
        )
    )
    
    container = ComposableNodeContainer(
        name='controller_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='assignment1_rt2',
                plugin='target_controller::TargetController',
                name='target_controller',
            )
        ],
        output='screen'
    )

    ui_node = Node(
        package='assignment1_rt2',
        executable='target_interface',
        name='target_interface',
        output='screen',
        prefix='xterm -e' #open node in another terminal
    )

    return LaunchDescription([
        bme_launch,
        container,
        ui_node
    ])