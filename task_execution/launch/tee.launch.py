"""Launch file for Task Execution Engine"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for TEE"""
    
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('task_execution'),
            'config',
            'tee_config.yaml'
        ]),
        description='Path to TEE configuration file'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # TEE Node
    tee_node = Node(
        package='task_execution',
        executable='tee_node',
        name='task_execution_engine',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_file')
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        config_file_arg,
        log_level_arg,
        tee_node
    ])
