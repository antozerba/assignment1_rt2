from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
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
    # compoenent_node = Node(
    #     package='assignment1_rt2',
    #     executable='controller_node',
    #     name='controller_node',
    #     output='screen',
    #     prefix='xterm -e' #open node in another terminal
    # )

    ui_node = Node(
        package='assignment1_rt2',
        executable='target_interface',
        name='target_interface',
        output='screen',
        prefix='xterm -e' #open node in another terminal
    )

    return LaunchDescription([
        container,
        # compoenent_node,
        ui_node
    ])