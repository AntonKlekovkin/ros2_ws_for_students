from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
import os
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    current_world = 'my_empty'
    robot_model_file_name = "model.xacro"
    
    pkg_m2wr = get_package_share_directory('m2wr_description')
    world_path = os.path.join(pkg_m2wr, 'worlds', current_world + '.sdf')

    urdf_path = PathJoinSubstitution([FindPackageShare("m2wr_description"), "urdf", robot_model_file_name])
    
    use_sim_time_arg = DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation time'
        )

    gz = ExecuteProcess(
            # Формируем команду одной строкой и включаем shell=True
            cmd=['gz sim -r -v 4 ' + world_path],
            output='screen',
            shell=True  # <--- КЛЮЧЕВОЙ ПАРАМЕТР
        )
    
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': Command(['xacro ', urdf_path])
            }
        ]
    )
    # jsp = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{
    #     'source_list': ['joint_states'],  # откуда брать данные
    #     'rate': 100  # частота публикации
    # }]
    # )
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            #'-world', current_world,
            '-name', 'm2wr',
            '-topic', 'robot_description',
            'allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0.2'
        ],
        output='screen'
    )
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_m2wr, 'config', 'ros_gz_bridge.yaml')            
        }]
        # arguments=[
        #     '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
        #     '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        # ]
    )

    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn]
    )
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz.')
                              
    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_m2wr, 'config', 'rviz_config.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        #gz,
        rsp,
        #jsp,
        delayed_spawn,
        #spawn,
        gz_bridge,
        rviz_arg,
        rviz 
    ])