import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    package_name='sphero_base'

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory('sphero_base'), 'config', 'ros_gz_bridge.yaml')            
        }]
        # arguments=[
        #     '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
        #     '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        # ]
    )

    spawn_table = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            #'-world', current_world,
            '-name', 'rails_system',
            '-file', os.path.join(get_package_share_directory('sphero_base'),'urdf','vibro_table.urdf'),
            'allow_renaming', 'true',
            '-x', '0.5', '-y', '0.0', '-z', '0.01'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gz_bridge,
        spawn_table
    ])