"""Launch file for complete system: TEE + FAL + MAVROS"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for complete system"""
    
    # Declare arguments
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@localhost:14557',
        description='MAVROS FCU URL'
    )
    
    # Include MAVROS launch (if available)
    # Note: This assumes you have MAVROS installed
    # mavros_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('mavros'),
    #             'launch',
    #             'px4.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'fcu_url': LaunchConfiguration('fcu_url')
    #     }.items()
    # )
    
    # Include FAL launch
    fal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('flight_abstraction'),
                'launch',
                'fal.launch.py'
            ])
        ])
    )
    
    # Include TEE launch
    tee_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('task_execution'),
                'launch',
                'tee.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        fcu_url_arg,
        # mavros_launch,  # Uncomment if MAVROS is available
        fal_launch,
        tee_launch
    ])
