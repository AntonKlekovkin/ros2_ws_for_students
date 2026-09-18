from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    current_world = 'maze'
        
    pkg_m2wr = get_package_share_directory('m2wr_description')
    world_path = os.path.join(pkg_m2wr, 'worlds', current_world + '.sdf')
    
    use_sim_time_arg = DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation time'
        )

    rviz_arg = DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz.')

    gz = ExecuteProcess(            
            cmd=['gz sim -r -v 4 ' + world_path],
            output='screen',
            shell=True  
        )
    
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_m2wr, 'config', 'rviz_config.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        gz,        
        rviz_arg,
        rviz 
    ])