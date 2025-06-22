from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    #declare arguments if needed
    use_navigan_arg = DeclareLaunchArgument(
        'use_navigan',
        default_value='false',
        description='Use NaviGAN controller.'
    )

    #NMPC controller node for recorded path
    nmpc_node = Node(
        package='nmpc_pkg',
        executable='nmpc_controller_node',
        name='nmpc_controller_node',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_navigan'))
    )

    #NMPC controller node for NaviGAN path
    navigan_node = Node(
        package='nmpc_pkg',
        executable='navigan_controller_node',
        name='navigan_controller_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_navigan'))
    )

    #log which controller is used
    log_nmpc = LogInfo(msg='Starting with recorded path-following NMPC controller...')
    log_navigan = LogInfo(msg='Starting with NaviGAN NMPC controller...')

    return LaunchDescription([
        use_navigan_arg,
        log_nmpc,
        log_navigan,
        nmpc_node,
        navigan_node,
    ])
