from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    vehicle_type = LaunchConfiguration('vehicle_type', default='ArduCopter')
    home_location = LaunchConfiguration('home', default='-35.363261,149.165230,584,353')
    
    # Start ArduPilot SITL
    sitl_process = ExecuteProcess(
        cmd=[
            'sim_vehicle.py',
            '-v', vehicle_type,
            '-L', home_location,
            '--no-mavproxy',  # We'll use MAVROS instead
            '--out', 'udp:127.0.0.1:14555',  # For MAVROS connection
            '--speedup', '1'
        ],
        shell=True,
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_type'),
        DeclareLaunchArgument('home'),
        sitl_process
    ])